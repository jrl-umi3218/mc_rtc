#include <mc_observers/FloatingBasePosVelObserver.h>

namespace mc_observers
{
FloatingBasePosVelObserver::FloatingBasePosVelObserver(const mc_rbdyn::Robot & controlRobot, double dt)
: FloatingBasePosObserver(controlRobot), dt_(dt), velFilter_(dt, /* cutoff period = */ 0.01)
{
}

void FloatingBasePosVelObserver::reset(const mc_rbdyn::Robot & realRobot, const sva::PTransformd & X_0_fb, const sva::MotionVecd & velW)
{
  FloatingBasePosObserver::reset(X_0_fb);
  FloatingBasePosObserver::run(realRobot);
  posWPrev_ = FloatingBasePosObserver::posW();;
  velW_ = velW;
  velFilter_.reset(velW);
}

void FloatingBasePosVelObserver::run(const mc_rbdyn::Robot & realRobot)
{
  FloatingBasePosObserver::run(realRobot);
  const sva::PTransformd posW = FloatingBasePosObserver::posW();
  sva::MotionVecd errVel = sva::transformError(posWPrev_, posW) / dt_;
  velFilter_.update(errVel);
  velW_ = velFilter_.vel();
  posWPrev_ = posW;
}

void FloatingBasePosVelObserver::updateRobot(mc_rbdyn::Robot & robot)
{
  FloatingBasePosObserver::updateRobot(robot);
  robot.velW(velW_);
}

void FloatingBasePosVelObserver::updateBodySensor(mc_rbdyn::Robot & realRobot, const std::string & sensorName)
{
  FloatingBasePosObserver::updateBodySensor(realRobot, sensorName);
  auto & sensor = realRobot.bodySensor(sensorName);
  sensor.linearVelocity(velW_.linear());
  sensor.angularVelocity(velW_.angular());
}

const sva::MotionVecd & FloatingBasePosVelObserver::velW() const
{
  return velW_;
}

} // namespace mc_observers
