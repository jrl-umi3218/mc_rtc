/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is inspired by Stephane's Caron original implementation as part of
 * lipm_walking_controller <https://github.com/stephane-caron/lipm_walking_controller>
 */

#pragma once

#include <mc_signal/ExponentialMovingAverage.h>

namespace mc_signal
{
/** Remove stationary offset from an input signal.
 *
 */
struct StationaryOffsetFilter
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
  StationaryOffsetFilter(double dt, double timeConstant, const Eigen::Vector3d & initValue = Eigen::Vector3d::Zero())
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
  void update(const Eigen::Vector3d & value)
  {
    average_.append(value);
    filteredValue_ = value - average_.eval();
    rawValue_ = value;
  }

  /** Get output value where the stationary offset has been filtered.
   *
   */
  const Eigen::Vector3d & eval() const
  {
    return filteredValue_;
  }

  /** Get raw value of input signal.
   *
   */
  const Eigen::Vector3d & raw() const
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
  Eigen::Vector3d filteredValue_;
  Eigen::Vector3d rawValue_;
  ExponentialMovingAverage average_;
};
} // namespace mc_signal
