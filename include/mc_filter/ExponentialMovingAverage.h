/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is inspired by Stephane's Caron original implementation as part of
 * lipm_walking_controller <https://github.com/stephane-caron/lipm_walking_controller>
 */

#pragma once
#include <mc_filter/utils/clamp.h>
#include <mc_rtc/logging.h>
#include <algorithm>
#include <cmath>

namespace mc_filter
{
/** Exponential Moving Average.
 *
 * This filter can be seen as an integrator:
 *
 * \f[
 *    y(t) = \frac{1}{T} \int_{u=0}^t x(u) e^{(u - t) / T} {\rm d}{\tau}
 * \f]
 *
 * with T > 0 a reset period acting as anti-windup. It can also (informally) be
 * interpreted as the average value of the input signal x(t) over the last T
 * seconds. Formally, it represents the amount of time for the smoothed
 * response of a unit input to reach 1-1/e (~63%) of the original signal.
 *
 * See <https://en.wikipedia.org/wiki/Exponential_smoothing>. It is equivalent
 * to a low-pass filter <https://en.wikipedia.org/wiki/Low-pass_filter> applied
 * to the integral of the input signal.
 *
 * Expects type VectorT to have:
 * - VectorT::Zero() static function
 * - VectorT::setZero()
 * Common types: Eigen::Vector3d, Eigen::Vector6d, etc.
 */
template<typename VectorT>
struct ExponentialMovingAverage
{
  /** Constructor.
   *
   * \param dt Time in [s] between two readings.
   *
   * \param timeConstant Informally, length of the recent-past window, in [s].
   *
   * \param initValue Initial value of the output average.
   *
   */
  ExponentialMovingAverage(double dt, double timeConstant, const VectorT & initValue = VectorT::Zero()) : dt_(dt)
  {
    this->reset(initValue);
    this->timeConstant(timeConstant);
  }

  /** Append a new reading to the series.
   *
   * \param value New value.
   */
  void append(const VectorT & value)
  {
    average_ += alpha_ * (value - average_);
    if(saturation_ > 0.)
    {
      utils::clampInPlace(average_, -saturation_, saturation_);
    }
  }

  /** Evaluate the smoothed statistic.
   *
   */
  const VectorT & eval() const
  {
    return average_;
  }

  /** Set output saturation; disable by providing a negative value.
   *
   * \param limit Output will saturate between -limit and +limit.
   */
  void saturation(double limit)
  {
    saturation_ = limit;
  }

  /** Reset average to provided value.
   *
   * \param initVal initial value of the average
   */
  void reset(const VectorT & initVal)
  {
    average_ = initVal;
  }

  /** Get time constant of the filter.
   *
   */
  double timeConstant() const
  {
    return timeConstant_;
  }

  /** Update time constant.
   *
   * \param T New time constant of the filter.
   *
   * \note T is explicitely enforced to respect the Nyquist–Shannon sampling theorem, that is T is at least 2*timestep.
   */
  void timeConstant(double T)
  {
    if(T < 2 * dt_)
    {
      mc_rtc::log::warning("Time constant must be at least twice the timestep (Nyquist–Shannon sampling theorem)");
      T = 2 * dt_;
    }
    alpha_ = 1. - std::exp(-dt_ / T);
    timeConstant_ = T;
  }

protected:
  VectorT average_ = VectorT::Zero();
  double alpha_;
  double dt_;
  double timeConstant_;
  double saturation_ = -1.;
};

} // namespace mc_filter
