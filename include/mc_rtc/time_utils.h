/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once
#include <chrono>

namespace mc_rtc
{
/**
 * @ class measure
 * @ brief Class to measure the execution time of a callable
 */
template<typename TimeT = std::chrono::duration<double, std::milli>,
         class ClockT = typename std::conditional<std::chrono::high_resolution_clock::is_steady,
                                                  std::chrono::high_resolution_clock,
                                                  std::chrono::steady_clock>::type>
struct measure
{
  /**
   * @brief Returns the quantity (count) of the elapsed time as TimeT units
   */
  template<typename F, typename... Args>
  /**
   * @brief Returns the execution time of the provide function
   *
   * @param func Function to evaluate
   * @param args Arguments to the function
   *
   * @return Time elapsed (in the provided unit)
   */
  static typename TimeT::rep execution(F && func, Args &&... args)
  {
    auto start = ClockT::now();
    func(std::forward<Args>(args)...);
    auto duration = std::chrono::duration_cast<TimeT>(ClockT::now() - start);
    return duration.count();
  }

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
  static TimeT duration(F && func, Args &&... args)
  {
    auto start = ClockT::now();
    func(std::forward<Args>(args)...);
    return std::chrono::duration_cast<TimeT>(ClockT::now() - start);
  }
};
using measure_s = measure<std::chrono::duration<double>>;
using measure_ms = measure<std::chrono::duration<double, std::milli>>;
using measure_ns = measure<std::chrono::duration<double, std::nano>>;

} // namespace mc_rtc
