/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once
#include <chrono>
#include <utility>

namespace mc_rtc
{
/**
 * @brief Class to measure the execution time of a callable
 */
template<typename TimeT = std::chrono::duration<double, std::milli>,
         class ClockT = typename std::conditional<std::chrono::high_resolution_clock::is_steady,
                                                  std::chrono::high_resolution_clock,
                                                  std::chrono::steady_clock>::type>
struct measure
{
  /**
   * @brief Returns the exectution time of the provided function (in chrono
   * type system)
   *
   * @param func Function to evaluate
   * @param args Arguments to the function
   *
   * @return Time elapsed
   */
  template<typename F, typename... Args>
  static TimeT execution(F && func, Args &&... args)
  {
    auto start = ClockT::now();
    func(std::forward<Args>(args)...);
    return std::chrono::duration_cast<TimeT>(ClockT::now() - start);
  }
};
using measure_s = measure<std::chrono::duration<double>>;
using measure_ms = measure<std::chrono::duration<double, std::milli>>;
using measure_us = measure<std::chrono::duration<double, std::micro>>;
using measure_ns = measure<std::chrono::duration<double, std::nano>>;

/**
 * @brief Class to measure execution time
 */
template<typename TimeT = std::chrono::duration<double, std::milli>,
         class ClockT = typename std::conditional<std::chrono::high_resolution_clock::is_steady,
                                                  std::chrono::high_resolution_clock,
                                                  std::chrono::steady_clock>::type>
struct Stopwatch
{
  using TimeP = typename TimeT::time_point;
  TimeP startTime_;

  Stopwatch()
  {
    startTime_ = ClockT::now();
  }

  inline void start()
  {
    startTime_ = ClockT::now();
  }

  inline TimeT elapsed()
  {
    auto elapsed = std::chrono::duration_cast<TimeT>(ClockT::now() - startTime_);
    return elapsed;
  }
};
using Stopwatch_s = Stopwatch<std::chrono::duration<double>>;
using Stopwatch_ms = Stopwatch<std::chrono::duration<double, std::milli>>;
using Stopwatch_us = Stopwatch<std::chrono::duration<double, std::micro>>;
using Stopwatch_ns = Stopwatch<std::chrono::duration<double, std::nano>>;

} // namespace mc_rtc
