/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is  inspired by Stephane's Caron implementation as part of
 * lipm_walking_controller <https://github.com/stephane-caron/lipm_walking_controller>
 */

#pragma once

namespace mc_signal
{

/** Low-pass filter from series of velocity measurements.
 *
 * Expects T to have:
 * - T::Zero() static method (e.g Eigen::Vector3d, etc)
 */
template<typename T>
struct LowPassFilter
{
  /** Constructor with cutoff period.
   *
   * \param dt Sampling period.
   *
   * \param period Cutoff period.
   *
   */
  LowPassFilter(double dt, double period = 0) : cutoffPeriod_(period), dt_(dt)
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
   * \note period is explicitely enforced to respect the Nyquist–Shannon sampling theorem, that is T is at least
   * 2*timestep.
   */
  void cutoffPeriod(double period)
  {
    period = std::max(period, 2 * dt_); // Nyquist–Shannon sampling theorem
    cutoffPeriod_ = period;
  }

  /** Reset position to an initial rest value.
   *
   * \param pos New position.
   *
   */
  void reset(const T & value)
  {
    eval_ = value;
  }

  /** Update velocity estimate from new position value.
   *
   * \param newPos New observed position.
   *
   */
  void update(const T & newValue)
  {
    double x = (cutoffPeriod_ <= dt_) ? 1. : dt_ / cutoffPeriod_;
    eval_ = x * newValue + (1. - x) * eval_;
  }

  /** Get filtered velocity.
   *
   */
  const T & eval() const
  {
    return eval_;
  }

  double dt() const
  {
    return dt_;
  }

private:
  T eval_;
  double cutoffPeriod_ = 0.;

protected:
  double dt_ = 0.005; // [s]
};

} // namespace mc_signal
