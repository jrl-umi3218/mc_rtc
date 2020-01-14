/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_filter/LowPass.h>

namespace mc_filter
{
/** Low-pass velocity filter from series of position measurements.
 *
 * Expects T to have:
 * - T::Zero() static method
 */
template<typename T>
struct LowPassFiniteDifferences : public LowPass<T>
{
  using LowPassT = LowPass<T>;

  /** Constructor with cutoff period.
   *
   * \param dt Sampling period.
   *
   * \param period Cutoff period.
   *
   */
  LowPassFiniteDifferences(double dt, double period) : LowPassT(dt, period)
  {
    LowPassT::reset(T::Zero(), T::Zero());
  }

  /** Reset filter to initial rest value.
   *
   * \param pos Initial position.
   * \param vel Initial velocity.
   */
  void reset(const T & pos, const T & vel)
  {
    LowPassT::reset(vel);
    prevValue_ = pos;
  }

  /** Update velocity estimate from new position value.
   *
   * \param newPos New observed position.
   *
   */
  void update(const T & newPos)
  {
    T discVel = (newPos - prevValue_) / LowPassT::dt();
    LowPassT::update(discVel);
    prevValue_ = newPos;
  }

  const T & prevValue() const
  {
    return prevValue_;
  }

protected:
  T prevValue_;

private:
  // Prevent calling the single-argument reset from parent's LowPass<T>
  using LowPass<T>::reset;
};
} // namespace mc_filter
