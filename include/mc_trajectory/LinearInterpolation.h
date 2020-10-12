/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

namespace mc_trajectory
{

template<typename T>
struct LinearInterpolation
{
  /**
   * @brief Linear interpolation between two values
   *
   * \f[
   *  f(t) = (1 - t) * v_1 + t * v_2;
   * \f]
   *
   * @tparam T value type, must support multiplication with scalar and addition with T
   * @param v1 First value
   * @param v2 Second value
   * @param t Interpolation ratio between 0 and 1
   *
   * @return
   * - v1 for t = 0
   * - v2 for t = 1
   * - value linearly interpolated between v1 and v2 otherwise
   */
  T operator()(const T & v1, const T & v2, double t) const
  {
    return (1 - t) * v1 + t * v2;
  }
};

} // namespace mc_trajectory
