/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_observers/Observer.h>
#include <mc_observers/api.h>
#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/log/Logger.h>
#include <mc_rtc/type_name.h>

namespace mc_control
{
struct MCController;
} // namespace mc_control

namespace mc_rbdyn
{
struct Robots;
}

namespace mc_observers
{

/**
 * @brief State observation pipeline
 *
 * Observers are responsible for estimating some of the robot properties from sensor measurements and/or fusing several
 * source of information (e.g the EncoderObserver estimates the joint position and velocity based on joint sensors, the
 * KinematicInertialObservers the state of the floating base from kinematics and IMU information, BodySensorObserver
 * uses BodySensor information to update the floating base, etc). The ObserverPipeline groups them into a \"State
 * observation pipeline\", that will run each observer sequentially. Some observers may be used to update the state of
 * the real robots instances used by the controller, while others may only be used for logging estimated values for
 * comparison purposes.
 */
struct MC_OBSERVERS_DLLAPI ObserverPipeline
{
  /** Holds an observer and its configuration within the ObserverPipeline */
  struct PipelineObserver
  {
    friend struct ObserverPipeline;

    PipelineObserver(const mc_observers::ObserverPtr & observer, const mc_rtc::Configuration & config)
    : observer_(observer)
    {
      config("update", update_);
      config("log", log_);
      config("gui", gui_);
      config("successRequired", successRequired_);
    }

    /* Const accessor to an observer generic interface
     *
     * @param name Name of the observer
     * @throws std::runtime_error if the observer is not part of the pipeline
     */
    const Observer & observer() const
    {
      return *observer_;
    }

    /* Non-const variant */
    Observer & observer()
    {
      return const_cast<Observer &>(static_cast<const PipelineObserver *>(this)->observer());
    }

    /** Get a an observer of type T
     *
     * \note Requires the code from which this is called to link against the
     * observer's library where type T is declared
     *
     * \tparam T type of the observer requested
     *
     * \throws If the observer does not exist or does not have the right type
     */
    template<typename T>
    const T & observer() const
    {
      auto ptr = dynamic_cast<T *>(observer_.get());
      if(!ptr)
      {
        mc_rtc::log::error_and_throw("{} observer type did not match the requested one: {}", observer_->type(),
                                     mc_rtc::type_name<T>());
      }
      return *ptr;
    }

    template<typename T>
    T & observer()
    {
      return const_cast<T &>(static_cast<const PipelineObserver *>(this)->observer<T>());
    }

    bool update() const noexcept
    {
      return update_;
    }

    bool log() const noexcept
    {
      return log_;
    }

    bool gui() const noexcept
    {
      return gui_;
    }

    /** Returns whether the last call to this observer succeeded */
    bool success() const noexcept
    {
      return success_;
    }

    /** Returns whether this observer must succeed or is allowed to fail
     * When true, the whole pipeline will fail if this observer fails. Otherwise
     * the pipeline will keep executing */
    bool successRequired() const noexcept
    {
      return successRequired_;
    }

  protected:
    ObserverPtr observer_ = nullptr; //< Observer
    bool update_ = true; //< Whether to update the real robot instance from this observer
    bool log_ = true; //< Whether to log this observer
    bool gui_ = true; //< Whether to display the gui
    bool successRequired_ = true; //< Whether this observer must succeed or is allowed to fail
    bool success_ = true; //< Whether this observer succeeded
  };

  ObserverPipeline(mc_control::MCController & ctl, const std::string & name);
  ObserverPipeline(mc_control::MCController & ctl);
  ~ObserverPipeline() = default;

  /* Load the observers */
  void create(const mc_rtc::Configuration & config, double dt);

  /* Initialize based on the current robot state */
  void reset();

  /* Run this observservation pipeline
   *
   * If an observer is unable to estimate the robot's state, it is expected to
   * return false. In this case, the pipeline execution is considered invalid,
   * and this status is reflected by the return value of this function. This
   * state may later be retrieved by success().
   *
   * @return True when the pipeline exectued properly
   * False otherwise (one or more observers failed to execute)
   **/
  bool run();

  /** @return True if the observers are running */
  inline bool runObservers() const noexcept
  {
    return runObservers_;
  }

  /**
   * @brief Whether to run the observers in this pipeline
   *
   * @param status True if the observers should be run
   */
  inline void runObservers(bool status)
  {
    runObservers_ = status;
  }

  /** @return True if the observers are updating the real robots instance. The
   * update does not occur if runObservers() is false. */
  inline bool updateObservers() const
  {
    return updateObservers_;
  }

  /**
   * @brief Whether to update the observers in this pipeline
   *
   * @param status True if the real robot instances should be update from the
   * observers's result. Update occurs only if runObservers() is true, and the
   * observer succeeded.
   */
  inline void updateObservers(bool status)
  {
    updateObservers_ = status;
  }

  /**
   * @brief Checks whether the last run of the pipeline succeeded
   *
   * @return True when the last call to run() succeeded
   */
  inline bool success() const noexcept
  {
    return success_;
  }

  /* Const accessor to an observer
   *
   * @param name Name of the observer
   * @throws std::runtime_error if the observer is not part of the pipeline
   */
  const PipelineObserver & observer(const std::string & name) const
  {
    auto it = std::find_if(pipelineObservers_.begin(), pipelineObservers_.end(),
                           [&name](const PipelineObserver & obs) { return obs.observer().name() == name; });
    if(it == pipelineObservers_.end())
    {
      mc_rtc::log::error_and_throw("Observer pipeline \"{}\" does not have any observer named \"{}\"", name_, name);
    }
    return *it;
  }

  /* Non-const variant */
  PipelineObserver & observer(const std::string & name)
  {
    return const_cast<PipelineObserver &>(static_cast<const ObserverPipeline *>(this)->observer(name));
  }

  const std::vector<PipelineObserver> & observers() const
  {
    return pipelineObservers_;
  }

  std::vector<PipelineObserver> & observers()
  {
    return pipelineObservers_;
  }

  /**
   * @brief Checks whether this pipeline has an observer
   *
   * @param name Name of the observer
   *
   * @return True if the observer is in the pipeline
   */
  bool hasObserver(const std::string & name) const
  {
    return std::find_if(pipelineObservers_.begin(), pipelineObservers_.end(),
                        [&name](const PipelineObserver & obs) { return obs.observer().name() == name; })
           != pipelineObservers_.end();
  }

  /**
   * @brief Checks if there is an observer of a specific type in the pipeline
   *
   * There may be more than one
   *
   * @param type Type of the observer
   *
   * @return True if there is at least one observer of this type in the pipeline
   */
  bool hasObserverType(const std::string & type) const
  {
    return std::find_if(pipelineObservers_.begin(), pipelineObservers_.end(),
                        [&type](const PipelineObserver & obs) { return obs.observer().type() == type; })
           != pipelineObservers_.end();
  }

  /*! \brief Short description of the pipeline */
  inline const std::string & desc() const noexcept
  {
    return desc_;
  }

  /* Name used to identify this pipeline */
  inline const std::string & name() const noexcept
  {
    return name_;
  }

  void addToLogger(mc_rtc::Logger &);
  void removeFromLogger(mc_rtc::Logger &);
  void addToGUI(mc_rtc::gui::StateBuilder &);
  void removeFromGUI(mc_rtc::gui::StateBuilder &);

protected:
  mc_control::MCController & ctl_;
  std::string name_ = {"DefaultObserverPipeline"}; ///< Name of this pipeline
  /* Short descriptive description of the observer used for CLI logging */
  std::string desc_ = {""};
  bool runObservers_ = true; ///< Whether to run this pipeline
  bool updateObservers_ = true; ///< Whether to update real robots from estimated state.
  bool success_ = false; ///< Whether the pipeline successfully executed

  /** Observers that will be run by the pipeline. */
  std::vector<PipelineObserver> pipelineObservers_;
};

} // namespace mc_observers
