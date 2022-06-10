/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <chrono>
#include <type_traits>

namespace mc_rtc
{

using duration_ms = std::chrono::duration<double, std::milli>;
using duration_us = std::chrono::duration<double, std::micro>;

/** mc_rtc::clock is a clock that is always steady and thus suitable for performance measurements */
using clock = typename std::conditional<std::chrono::high_resolution_clock::is_steady,
                                        std::chrono::high_resolution_clock,
                                        std::chrono::steady_clock>::type;

} // namespace mc_rtc
