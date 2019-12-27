/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is inspired by Stephane's Caron original implementation as part of
 * lipm_walking_controller <https://github.com/stephane-caron/lipm_walking_controller>
 */

#pragma once

#include <mc_filter/ExponentialMovingAverage.h>

namespace mc_filter
{
/** Remove stationary offset from an input signal.
 *
 * Expects VectorT to act as a vector (typically Eigen::Vector3d), supporting:
 * - VectorT::Zero() static member function
 * - VectorT::setZero() member
 * - VectorT should also respect the requirements of ExponentialMovingAverage
 */
template<typename VectorT>
struct StationaryOffset
{
  /** Constructor.
   *
   * \param dt Time in [s] between two readings.
   *
   * \param timeConstant Length of recent-past window used to evaluate the
   * stationary offset.
   *
   * \param initValue Initial value of the input signal.
   *
   */
  StationaryOffset(double dt, double timeConstant, const VectorT & initValue = VectorT::Zero())
  : average_(dt, timeConstant, initValue)
  {
    filteredValue_ = initValue;
    rawValue_ = initValue;
  }

  /** Update input signal value.
   *
   * \param value New value.
   *
   */
  void update(const VectorT & value)
  {
    average_.append(value);
    filteredValue_ = value - average_.eval();
    rawValue_ = value;
  }

  /** Get output value where the stationary offset has been filtered.
   *
   */
  const VectorT & eval() const
  {
    return filteredValue_;
  }

  /** Get raw value of input signal.
   *
   */
  const VectorT & raw() const
  {
    return rawValue_;
  }

  /** Reset everything to zero.
   *
   */
  void setZero()
  {
    average_.setZero();
    filteredValue_.setZero();
    rawValue_.setZero();
  }

  /** Get time constant of the filter.
   *
   */
  double timeConstant() const
  {
    return average_.timeConstant();
  }

  /** Update time constant.
   *
   * \param T New time constant of the filter.
   *
   */
  void timeConstant(double T)
  {
    average_.timeConstant(T);
  }

private:
  VectorT filteredValue_;
  VectorT rawValue_;
  ExponentialMovingAverage<VectorT> average_;
};
} // namespace mc_filter
