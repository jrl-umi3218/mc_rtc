/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is inspired by Stephane's Caron original implementation as part of
 * lipm_walking_controller <https://github.com/stephane-caron/lipm_walking_controller>
 */

#pragma once

#include <mc_filter/utils/clamp.h>

namespace mc_filter
{
/** Leaky integrator.
 *
 * The output satisfies the differential equation:
 *
 * \f[
 *     \dot{y}(t) = x(t) - leakRate * y(t)
 * \f]
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
  LeakyIntegrator() : integral_(VectorT::Zero()), rate_(0.1), saturation_(-1.) {}

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
      utils::clampInPlace(integral_, -saturation_, saturation_);
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
  inline void reset()
  {
    integral_.setZero();
  }

private:
  VectorT integral_;
  double rate_;
  double saturation_;
};
} // namespace mc_filter
