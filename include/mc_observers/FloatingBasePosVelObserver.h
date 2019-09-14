#include <mc_observers/FloatingBasePosObserver.h>
#include <mc_observers/LowPassVelocityFilter.h>

namespace mc_observers
{
struct MC_OBSERVERS_DLLAPI FloatingBasePosVelObserver : public FloatingBasePosObserver
{
  FloatingBasePosVelObserver(const mc_rbdyn::Robot & controlRobot, double dt);
  void reset(const mc_rbdyn::Robot & realRobot, const sva::PTransformd & X_0_fb, const sva::MotionVecd & velW);
  void run(const mc_rbdyn::Robot & realRobot_);
  void updateRobot(mc_rbdyn::Robot & robot);
  void updateBodySensor(mc_rbdyn::Robot & robot, const std::string & sensorName = "FloatingBase");

  const sva::MotionVecd & velW() const;

private:
  /** Controller timestep */
  double dt_;
  /** Previous estimated position.
   * Used to compute finite differences estimation of the velocity */
  sva::PTransformd posWPrev_;

  /**
   * Estimated velocity through finite differences and low-pass filtering
   **/
  LowPassVelocityFilter<sva::MotionVecd> velFilter_;
  sva::MotionVecd velW_;

private:
  /** Prevent from resetting only the position */
  using FloatingBasePosObserver::reset;
};
} // namespace mc_observers
