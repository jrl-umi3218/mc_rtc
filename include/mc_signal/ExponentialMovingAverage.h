/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is inspired by Stephane's Caron original implementation as part of
 * lipm_walking_controller <https://github.com/stephane-caron/lipm_walking_controller>
 */

#pragma once

namespace mc_signal
{
/** Exponential Moving Average.
 *
 * This filter can be seen as an integrator:
 *
 *    y(t) = 1/T int_{u=0}^t x(u) e^{(u - t) / T} d{u}
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
 */
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
  ExponentialMovingAverage(double dt, double timeConstant, const Eigen::Vector3d & initValue = Eigen::Vector3d::Zero())
  : dt_(dt)
  {
    average_ = initValue;
    this->timeConstant(timeConstant);
  }

  /** Append a new reading to the series.
   *
   * \param value New value.
   *
   */
  void append(const Eigen::Vector3d & value)
  {
    average_ += alpha_ * (value - average_);
    if(saturation_ > 0.)
    {
      saturate_();
    }
  }

  /** Evaluate the smoothed statistic.
   *
   */
  const Eigen::Vector3d & eval() const
  {
    return average_;
  }

  /** Set output saturation; disable by providing a negative value.
   *
   * \param limit Output will saturate between -limit and +limit.
   *
   */
  void saturation(double limit)
  {
    saturation_ = limit;
  }

  /** Reset average to zero.
   *
   */
  void setZero()
  {
    average_.setZero();
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
   */
  void timeConstant(double T)
  {
    T = std::max(T, 2 * dt_);
    alpha_ = 1. - std::exp(-dt_ / T);
    timeConstant_ = T;
  }

private:
  /** Saturate averaged values.
   *
   */
  void saturate_()
  {
    for(unsigned i = 0; i < 3; i++)
    {
      if(average_(i) < -saturation_)
      {
        average_(i) = -saturation_;
      }
      else if(average_(i) > saturation_)
      {
        average_(i) = saturation_;
      }
    }
  }

protected:
  Eigen::Vector3d average_ = Eigen::Vector3d::Zero();
  double alpha_;
  double dt_;
  double timeConstant_;
  double saturation_ = -1.;
};

} // namespace mc_signal
