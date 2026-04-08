/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <chrono>
#include <type_traits>
#include <utility>

namespace mc_rtc
{

using duration_ms = std::chrono::duration<double, std::milli>;
using duration_us = std::chrono::duration<double, std::micro>;

/** mc_rtc::clock is a clock that is always steady and thus suitable for performance measurements */
using clock = typename std::conditional<std::chrono::high_resolution_clock::is_steady,
                                        std::chrono::high_resolution_clock,
                                        std::chrono::steady_clock>::type;

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
static mc_rtc::duration_ms timedExecution(F && func, Args &&... args)
{
  auto start = mc_rtc::clock::now();
  func(std::forward<Args>(args)...);
  return std::chrono::duration_cast<mc_rtc::duration_ms>(mc_rtc::clock::now() - start);
}

inline double duration_ms_count(std::chrono::time_point<mc_rtc::clock> start,
                                std::chrono::time_point<mc_rtc::clock> end)
{
  return mc_rtc::duration_ms(end - start).count();
}

inline mc_rtc::duration_ms elapsed_ms(std::chrono::time_point<mc_rtc::clock> start)
{
  return mc_rtc::duration_ms(mc_rtc::clock::now() - start);
}

inline double elapsed_ms_count(std::chrono::time_point<mc_rtc::clock> start)
{
  return mc_rtc::elapsed_ms(start).count();
}
} // namespace mc_rtc
