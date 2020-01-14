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
    this->reset(initValue);
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
    if(saturation_ > 0.)
    {
      utils::clampInPlace(filteredValue_, -saturation_, saturation_);
    }
  }

  /** Get output value where the stationary offset has been filtered.
   *
   */
  const VectorT & eval() const
  {
    return filteredValue_;
  }

  /** Reset everything to an initial value
   *
   * \param initValue Initial value from which to restart the filter.
   */
  void reset(const VectorT & initValue)
  {
    average_.reset(initValue);
    filteredValue_ = initValue;
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

  /** Set output saturation; disable by providing a negative value.
   *
   * \param limit Output will saturate between -limit and +limit.
   */
  void saturation(double limit)
  {
    saturation_ = limit;
  }

private:
  VectorT filteredValue_ = VectorT::Zero();
  ExponentialMovingAverage<VectorT> average_;
  double saturation_ = -1;
};
} // namespace mc_filter
