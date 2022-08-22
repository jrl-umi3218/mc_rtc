#pragma once

#include <mc_rtc/logging.h>
#include <algorithm>

namespace mc_filter
{

/**Compose filter of Low-pass and High-pass filter from two series of measurements.
 *
 * Expects T to have:
 * - T::Zero() static method (e.g Eigen::Vector3d, etc)
 */
template<typename T>
struct LowPassCompose
{
  /** Constructor with cutoff period.
   *
   * \param dt Sampling period.
   *
   * \param period Cutoff period.
   *
   */
  LowPassCompose(double dt, double period = 0)
  {
    cutoffPeriod(period);
    dt_ = dt;
    reset(T::Zero());
  }

  /** Get cutoff period of the high pass filter. */
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
    if(period < 2 * dt_)
    {
      mc_rtc::log::warning("Time constant must be at least twice the timestep (Nyquist–Shannon sampling theorem)");
      period = 2 * dt_;
    }
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
    eval_hp_ = eval_;
    eval_lp_ = eval_;
  }

  /** Update estimate from new two sources
   *
   * \param newValue_hp New observed value filtered throught high pass.
   *
   * \param newValue_lp New observed value filtered throught low pass.
   *
   */
  void update(const T & newValue_hp, const T & newValue_lp)
  {
    double x = (cutoffPeriod_ <= dt_) ? 1. : dt_ / cutoffPeriod_;

    // Update the low pass filter
    eval_lp_ = x * newValue_lp + (1. - x) * eval_lp_;

    // Update the high pass filter output
    T eval_hp = (newValue_hp - eval_hp_);

    // Summation of both components
    eval_ = eval_hp + eval_lp_;

    // Update high pass filter state
    eval_hp_ = (x * newValue_hp + (1. - x) * eval_hp_);

    // Use for logs
    value_hp_ = newValue_hp;
    value_lp_ = newValue_lp;
  }

  /** Get filtered velocity.
   *
   */
  const T & eval() const
  {
    return eval_;
  }

  const T & input_lp() const
  {
    return value_lp_;
  }
  const T & input_hp() const
  {
    return value_hp_;
  }

  /** Get sampling period.
   *
   */
  double dt() const
  {
    return dt_;
  }

  /** Set sampling period.
   *
   * \param dt Sampling period.
   *
   * \note the cutoff period is updated to satisfy the Nyquist–Shannon sampling theorem according the new sampling
   * period.
   */
  void dt(double dt)
  {
    dt_ = dt;
    cutoffPeriod(cutoffPeriod_);
  }

private:
  T eval_;
  T eval_lp_;
  T eval_hp_;
  T value_hp_;
  T value_lp_;
  double cutoffPeriod_ = 0.;

protected:
  double dt_ = 0.005; // [s]
};

} // namespace mc_filter