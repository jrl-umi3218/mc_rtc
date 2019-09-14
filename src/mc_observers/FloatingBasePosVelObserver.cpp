#include <mc_observers/FloatingBasePosVelObserver.h>

namespace mc_observers
{
FloatingBasePosVelObserver::FloatingBasePosVelObserver(const mc_rbdyn::Robot & controlRobot, double dt)
: FloatingBasePosObserver(controlRobot), dt_(dt), velFilter_(dt, /* cutoff period = */ 0.01)
{
}

void FloatingBasePosVelObserver::reset(const sva::PTransformd & X_0_fb, const sva::MotionVecd & velW)
{
  FloatingBasePosObserver::reset(X_0_fb);
  posWPrev_ = X_0_fb;
  velW_ = velW;
  velFilter_.reset(velW);
}

void FloatingBasePosVelObserver::run(const mc_rbdyn::Robot & realRobot)
{
  FloatingBasePosObserver::run(realRobot);
  const sva::PTransformd posW = FloatingBasePosObserver::posW();
  sva::MotionVecd err = sva::transformError(posWPrev_, posW);
  velFilter_.update(err);
  velW_ = velFilter_.vel();
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

} // namespace mc_observers
