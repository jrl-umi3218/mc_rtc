/*
 * Copyright 2025 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once
#include <mc_rtc/clock.h>
#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/log/Logger.h>
#include <future>
#include <optional>

namespace mc_rtc::threading
{

/**
 * @brief Helper base class for asynchronous jobs using CRTP.
 *
 * This class provides a generic interface for running asynchronous computations,
 * managing their state, and adding logging and GUI.
 * Child classes must inherit using CRTP and implement required methods.
 *
 * Upon destruction, any associated GUI elements and log entries are automatically removed.
 *
 * The `input_` member may be modified when the async task is not running (i.e., before calling startAsync or when
 * running() is false). The input should not be modified while the async task is running (running() is true). Use
 * input() to access it. Note that while the input() method implements a safeguard that throws if the job is running, it
 * does not inherently make modifying the input in-place safe, ensure that your code does not start another async job
 * while you are modifying the input.
 *
 * @note You must call checkResult() at every iteration of your controller loop. This method is responsible for
 * bookkeeping, result retrieval, and deferred GUI/logging actions. It is roughly equivalent to \c ros::spinOnce()
 * for ROS nodes.
 *
 * @tparam Derived The child class type (CRTP).
 * @tparam Input The input type for the job.
 * @tparam Result The result type produced by the job.
 *
 * @section RequiredMethods Required/Optional Child Class Methods
 * - Result computeJob()
 *   - Implements the actual computation. Called asynchronously. (Required)
 * - void addToLoggerImpl()
 *   - Adds job-specific log entries after the first result is available. (Optional)
 *     @note Will only be called if the user has previously called addToLogger().
 * - void addToGUIImpl()
 *   - Adds job-specific GUI elements after the first result is available. (Optional)
 *     @note Will only be called if the user has previously called addToGUI().
 *
 * @section UsageExample Example Usage
 * @code
 * struct MyAsyncJob : public mc_rtc::MakeAsyncJob<MyAsyncJob, MyInput, MyResult>
 * {
 *   MyResult computeJob()
 *   {
 *     MyResult res;
 *     res.value = input.data * 2;
 *     return res;
 *   }
 *
 *   // Optional: add logging
 *   void addToLoggerImpl()
 *   {
 *     logger_->addLogEntry("my_job_value", this, [this]() { return lastResult_->value; });
 *   }
 *
 *   // Optional: add GUI
 *   void addToGUIImpl()
 *   {
 *     gui_->addElement(this, guiCategory_, mc_rtc::gui::Label("Result", [this]() { return lastResult_->value; }));
 *   }
 * };
 *
 * // Usage:
 * MyAsyncJob job;
 * job.input = ...;
 * job.startAsync();
 * // Later, check for completion:
 * if(job.checkResult())
 * {
 *   auto result = *job.lastResult();
 * }
 * // To enable deferred logger/GUI calls:
 * job.addToLogger(logger, "prefix");
 * job.addToGUI(gui, {"Category"});
 * // Upon destruction, GUI and logging entries are automatically removed.
 * @endcode
 */
template<typename Derived, typename Input, typename Result>
struct AsyncJob
{
  struct Timers
  {
    double dt_startAsync = 0;
    double dt_checkResult = 0;
    double dt_loggerImpl = 0;
    double dt_guiImpl = 0;

    // Async loggers
    // Technically not guaranteed to be portable on all plateforms, but should be fine for the ones we care about
    std::atomic<double> dt_compute{0};
  };

  AsyncJob() = default;
  AsyncJob(const AsyncJob &) = delete;
  AsyncJob & operator=(const AsyncJob &) = delete;
  AsyncJob(AsyncJob &&) = delete;
  AsyncJob & operator=(AsyncJob &&) = delete;

  ~AsyncJob()
  {
    removeFromLogger();
    removeFromGUI();
  }

  /**
   * @brief Access the input data for the job.
   *
   * Returns a writable reference to the input. Throws std::runtime_error if the async job is running.
   * Only modify the input when running() is false.
   *
   * @return Reference to the input data.
   * @throws std::runtime_error if the job is running.
   */
  Input & input()
  {
    if(running_)
    {
      throw std::runtime_error("AsyncJob: cannot get write-access to the input while an async job is running. Please "
                               "ensure that running() is false before modifying the input in place");
    }
    return input_;
  }

  void startAsync()
  {
    auto start_async = mc_rtc::clock::now();
    futureResult_ = std::async(
        [this]()
        {
          auto start_compute = mc_rtc::clock::now();
          auto result = derived().computeJob();
          timers_.dt_compute = mc_rtc::elapsed_ms_count(start_compute);
          return result;
        });
    running_ = true;
    startedOnce_ = true;
    canGetSharedState_ = true;
    timers_.dt_startAsync = mc_rtc::elapsed_ms_count(start_async);
  }

  bool running() const noexcept { return running_; }

  /**
   * @return True if the job has been started at least once.
   */
  bool startedOnce() const noexcept { return startedOnce_; }

  /**
   * @brief Check if the asynchronous job has completed and handle bookkeeping.
   *
   * This method must be called at every iteration of the controller.
   * It is responsible for all bookkeeping related to the async job:
   * - Checks if the asynchronous computation has finished.
   * - Retrieves and stores the result if available.
   * - Calls deferred GUI and logging methods if enabled.
   *
   * This is roughly speaking the equivalent to \c ros::spinOnce() for ROS nodes.
   *
   * @return True if the job has completed and a result is available, false otherwise.
   */
  bool checkResult()
  {
    if(!running_) return false;
    auto start_check = mc_rtc::clock::now();
    if(futureResult_.wait_for(std::chrono::seconds(0)) == std::future_status::ready && futureResult_.valid()
       && canGetSharedState_)
    {

      try
      {
        lastResult_ = futureResult_.get();
      }
      catch(const std::exception & e)
      {
        mc_rtc::log::error("[{}] Exception in async job: {}", derived().name(), e.what());
        running_ = false;
        canGetSharedState_ = false;
        return false;
      }

      canGetSharedState_ = false; // prevent invalid multiple calls to future::get()
      if(logger_ && !inLogger_)
      {
        timers_.dt_loggerImpl = mc_rtc::timedExecution([this]() { addToLogger_(); }).count();
        inLogger_ = true;
      }
      else
      {
        timers_.dt_loggerImpl = 0;
      }

      if(gui_ && !inGUI_)
      {
        timers_.dt_guiImpl = mc_rtc::timedExecution([this]() { addToGUI_(); }).count();
        inGUI_ = true;
      }
      else
      {
        timers_.dt_guiImpl = 0;
      }
      running_ = false;
      return true;
    }
    timers_.dt_checkResult = mc_rtc::elapsed_ms_count(start_check);
    return false;
  }

  const std::optional<Result> & lastResult() const noexcept { return lastResult_; }

  void addToLogger(mc_rtc::Logger & logger, const std::string & prefix)
  {
    if(logger_ && inLogger_) return;
    logger_ = &logger;
    loggerPrefix_ = prefix;
  }

  void addToGUI(mc_rtc::gui::StateBuilder & gui, const std::vector<std::string> & category)
  {
    if(gui_ && inGUI_) return;
    gui_ = &gui;
    guiCategory_ = category;
  }

  void removeFromLogger()
  {
    if(logger_)
    {
      logger_->removeLogEntries(this);
      inLogger_ = false;
    }
  }

  void removeFromGUI()
  {
    if(gui_)
    {
      gui_->removeElements(this);
      inGUI_ = false;
    }
  }

  std::string name() const { return "AsyncJob"; }

protected: // bookkeeping for the async job
  std::atomic<bool> running_ = false;
  bool inLogger_ = false;
  bool inGUI_ = false;
  bool startedOnce_ = false;

  std::future<Result> futureResult_;
  // true if futureResult_.get() has not yet been called, false otherwise
  bool canGetSharedState_ = false;

  std::optional<Result> lastResult_;
  Timers timers_;

  /**
   * @brief Input data for the asynchronous job.
   *
   * This member holds the input required for the job computation.
   * It can be modified only when the async job is not running (i.e., before calling startAsync() or when running() is
   * false). Do not modify this while the async job is running. Use the input() accessor to obtain a writable reference,
   * which throws if the job is running.
   */
  Input input_;

  // CRTP: derived must implement this
  // Result computeJob();

  // CRTP: derived may implement these
  void addToLoggerImpl() {}
  void addToGUIImpl() {}

  // Store context for logging/GUI
  mc_rtc::Logger * logger_ = nullptr;
  std::string loggerPrefix_ = "";
  mc_rtc::gui::StateBuilder * gui_ = nullptr;
  std::vector<std::string> guiCategory_;

protected:
  Derived & derived() { return static_cast<Derived &>(*this); }

  void addToLogger_()
  {
    auto contactPrefix = "perf_" + loggerPrefix_ + "_";
    logger_->addLogEntry(contactPrefix + "startAsync [ms]", this, [this]() { return timers_.dt_startAsync; });
    logger_->addLogEntry(contactPrefix + "checkResult [ms]", this, [this]() { return timers_.dt_checkResult; });
    logger_->addLogEntry(contactPrefix + "loggerImpl [ms]", this, [this]() { return timers_.dt_loggerImpl; });
    logger_->addLogEntry(contactPrefix + "guiImpl [ms]", this, [this]() { return timers_.dt_guiImpl; });
    logger_->addLogEntry(contactPrefix + "async_compute [ms]", this, [this]() { return timers_.dt_compute.load(); });
    derived().addToLoggerImpl();
  }

  void addToGUI_() { derived().addToGUIImpl(); }
};

// Helper alias for CRTP inheritance
template<typename Derived, typename Input, typename Result>
using MakeAsyncJob = AsyncJob<Derived, Input, Result>;
} // namespace mc_rtc::threading
