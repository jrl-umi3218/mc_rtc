/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is  inspired by Stephane's Caron implementation as part of
 * lipm_walking_controller <https://github.com/stephane-caron/lipm_walking_controller>
 */

#pragma once

namespace mc_signal
{

/** Low-pass velocity filter from series of velocity measurements.
 *
 * Expects T to have:
 * - T::Zero() static method (e.g Eigen::Vector3d, etc)
 */
template<typename T>
struct LowPassVelocityFilter
{
  /** Constructor with cutoff period.
   *
   * \param dt Sampling period.
   *
   * \param period Cutoff period.
   *
   */
  LowPassVelocityFilter(double dt, double period = 0) : cutoffPeriod_(period), dt_(dt)
  {
    reset(T::Zero());
  }

  /** Get cutoff period.
   *
   */
  double cutoffPeriod() const
  {
    return cutoffPeriod_;
  }

  /** Set cutoff period.
   *
   * \param period New cutoff period.
   *
   */
  void cutoffPeriod(double period)
  {
    cutoffPeriod_ = period;
  }

  /** Reset position to an initial rest value.
   *
   * \param pos New position.
   *
   */
  void reset(T vel)
  {
    vel_ = vel;
  }

  /** Update velocity estimate from new position value.
   *
   * \param newPos New observed position.
   *
   */
  void update(const T & newVel)
  {
    double x = (cutoffPeriod_ <= dt_) ? 1. : dt_ / cutoffPeriod_;
    vel_ = x * newVel + (1. - x) * vel_;
  }

  /** Get filtered velocity.
   *
   */
  const T & vel()
  {
    return vel_;
  }

private:
  T vel_;
  double cutoffPeriod_ = 0.;

protected:
  double dt_ = 0.005; // [s]
};

} // namespace mc_signal
