/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is inspired by Stephane's Caron original implementation as part of
 * lipm_walking_controller <https://github.com/stephane-caron/lipm_walking_controller>
 */

#pragma once

namespace mc_signal
{
/** Leaky integrator.
 *
 * The output satisfies the differential equation:
 *
 *     yd(t) = x(t) - leakRate * y(t)
 *
 * A leaky integrator is implemented exactly as an exponential moving average,
 * but it is homogeneous to the integral of the input signal (rather than the
 * signal itself). See <https://en.wikipedia.org/wiki/Leaky_integrator>.
 *
 * Expects VectorT to act as a vector (typically Eigen::Vector3d), supporting:
 * - VectorT::Zero() static member function
 * - VectorT::setZero() member
 * - Component-wise math operator *
 * - Access operator ()
 * - VectorT::size()
 */
template<typename VectorT>
struct LeakyIntegrator
{
  /** Add constant input for a fixed duration.
   *
   * \param value Constant input.
   *
   * \param dt Fixed duration.
   *
   */
  inline void add(const VectorT & value, double dt)
  {
    integral_ = (1. - rate_ * dt) * integral_ + dt * value;
    if(saturation_ > 0.)
    {
      saturate();
    }
  }

  /** Evaluate the output of the integrator.
   *
   */
  inline const VectorT & eval() const
  {
    return integral_;
  }

  /** Get leak rate.
   *
   */
  inline double rate() const
  {
    return rate_;
  }

  /** Set the leak rate of the integrator.
   *
   * \param rate New leak rate.
   *
   */
  inline void rate(double rate)
  {
    rate_ = rate;
  }

  /** Set output saturation. Disable by providing a negative value.
   *
   * \param s Output will saturate between -s and +s.
   *
   */
  inline void saturation(double s)
  {
    saturation_ = s;
  }

  /** Reset integral to zero.
   *
   */
  inline void setZero()
  {
    integral_.setZero();
  }

private:
  inline void saturate()
  {
    for(unsigned i = 0; i < integral_.size(); i++)
    {
      if(integral_(i) < -saturation_)
      {
        integral_(i) = -saturation_;
      }
      else if(integral_(i) > saturation_)
      {
        integral_(i) = saturation_;
      }
    }
  }

private:
  VectorT integral_ = VectorT::Zero();
  double rate_ = 0.1;
  double saturation_ = -1.;
};
} // namespace mc_signal
