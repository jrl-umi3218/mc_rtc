/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_signal/LowPassVelocityFilter.h>

namespace mc_signal
{
/** Low-pass velocity filter from series of position measurements.
 *
 * Expects T to have:
 * - T::Zero() static method
 */
template<typename T>
struct LowPassFiniteDifferencesVelocityFilter : public LowPassVelocityFilter<T>
{
  /** Constructor with cutoff period.
   *
   * \param dt Sampling period.
   *
   * \param period Cutoff period.
   *
   */
  LowPassFiniteDifferencesVelocityFilter(double dt, double period = 0) : LowPassVelocityFilter<T>(dt, period)
  {
    reset(T::Zero());
  }

  /** Reset position to an initial rest value.
   *
   * \param pos New position.
   *
   */
  void reset(T pos)
  {
    LowPassVelocityFilter<T>::reset(T::Zero());
    pos_ = pos;
  }

  /** Reset position to an initial rest value.
   *
   * \param pos New position.
   * \param vel New velocity.
   */
  void reset(T pos, T vel)
  {
    LowPassVelocityFilter<T>::reset(vel);
    pos_ = pos;
  }

  /** Update velocity estimate from new position value.
   *
   * \param newPos New observed position.
   *
   */
  void update(const T & newPos)
  {
    T discVel = (newPos - pos_) / dt_;
    pos_ = newPos;
    LowPassVelocityFilter<T>::update(discVel);
  }

  /** Update position only.
   *
   */
  void updatePositionOnly(const T & newPos)
  {
    pos_ = newPos;
  }

private:
  T pos_;
};
} // namespace mc_signal
