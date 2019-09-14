#include <mc_observers/LowPassVelocityFilter.h>

namespace mc_observers
{
/** Low-pass velocity filter from series of position measurements.
 *
 */
template<typename T>
struct LowPassVelocityFilterFiniteDifferences : public LowPassVelocityFilter<T>
{
  /** Constructor.
   *
   * \param dt Sampling period.
   *
   */
  LowPassFiniteDifferencesVelocityFilter(double dt) : LowPassVelocityFilter<T>(dt)
  {
    reset(T::Zero());
  }

  /** Constructor with cutoff period.
   *
   * \param dt Sampling period.
   *
   * \param period Cutoff period.
   *
   */
  LowPassFiniteDifferencesVelocityFilter(double dt, double period) : LowPassVelocityFilter<T>(dt, period)
  {
    reset(T::Zero());
    LowPassVelocityFilter<T>::cutoffPeriod(period);
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
  using LowPassVelocityFilter<T>::dt_;
};
} // namespace mc_observers
