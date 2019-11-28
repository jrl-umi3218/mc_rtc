/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_signal/LowPassFilter.h>

namespace mc_signal
{
/** Low-pass velocity filter from series of position measurements.
 *
 * Expects T to have:
 * - T::Zero() static method
 */
template<typename T>
struct LowPassFiniteDifferencesVelocityFilter : public LowPassFilter<T>
{
  using LowPassFilterT = LowPassFilter<T>;

  /** Constructor with cutoff period.
   *
   * \param dt Sampling period.
   *
   * \param period Cutoff period.
   *
   */
  LowPassFiniteDifferencesVelocityFilter(double dt, double period) : LowPassFilterT(dt, period)
  {
    LowPassFilterT::reset(T::Zero());
    prevValue_ = T::Zero();
  }

  /** Reset filter to initial rest value.
   *
   * \param pos Initial position.
   * \param vel Initial velocity.
   */
  void reset(T pos, T vel)
  {
    LowPassFilterT::reset(vel);
    prevValue_ = pos;
  }

  /** Update velocity estimate from new position value.
   *
   * \param newPos New observed position.
   *
   */
  void update(const T & newPos)
  {
    T discVel = (newPos - prevValue_) / LowPassFilterT::dt();
    LowPassFilterT::update(discVel);
    prevValue_ = newPos;
  }

  const T & prevValue() const
  {
    return prevValue_;
  }

protected:
  T prevValue_;

private:
  // Prevent calling the single-argument reset from parent's LowPassFilter<T>
  using LowPassFilter<T>::reset;
};
} // namespace mc_signal
