#include "FloatingBasePosVelObserver.h"

namespace mc_observers
{
FloatingBasePosVelObserver::FloatingBasePosVelObserver(const std::string & name,
                                                       double dt,
                                                       const mc_rtc::Configuration & config)
: FloatingBasePosObserver(name, dt, config), velFilter_(dt, /* cutoff period = */ 0.01)
{
  LOG_SUCCESS("FloatingBasePosVelObserver created with dt " << FloatingBasePosObserver::dt());
}

FloatingBasePosVelObserver::~FloatingBasePosVelObserver() {}

void FloatingBasePosVelObserver::reset(const mc_rbdyn::Robot & realRobot)
{
  reset(realRobot, realRobot.velW());
}

void FloatingBasePosVelObserver::reset(const mc_rbdyn::Robot & realRobot, const sva::MotionVecd & velW)
{
  FloatingBasePosObserver::reset(realRobot);
  posWPrev_ = FloatingBasePosObserver::posW();
  velW_ = velW;
  velFilter_.reset(velW);
}

bool FloatingBasePosVelObserver::run(const mc_rbdyn::Robot & realRobot)
{
  FloatingBasePosObserver::run(realRobot);
  const sva::PTransformd posW = FloatingBasePosObserver::posW();
  LOG_INFO("dt: " << dt());
  sva::MotionVecd errVel = sva::transformError(posWPrev_, posW) / dt();
  velFilter_.update(errVel);
  velW_ = velFilter_.vel();
  posWPrev_ = posW;
  return true;
}

void FloatingBasePosVelObserver::updateRobot(mc_rbdyn::Robot & realRobot)
{
  FloatingBasePosObserver::updateRobot(realRobot);
  realRobot.velW(velW_);
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

void FloatingBasePosVelObserver::addToLogger(mc_rtc::Logger & logger)
{
  FloatingBasePosObserver::addToLogger(logger);
  logger.addLogEntry("observer_" + name() + "_velW", [this]() { return velW_; });
}
void FloatingBasePosVelObserver::removeFromLogger(mc_rtc::Logger & logger)
{
  FloatingBasePosObserver::removeFromLogger(logger);
  logger.removeLogEntry("observer_" + name() + "_velW");
}
void FloatingBasePosVelObserver::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  FloatingBasePosObserver::addToGUI(gui);
  gui.addElement({"Observers", name()}, mc_rtc::gui::Arrow("Velocity", [this]() { return posW().translation(); },
                                                           [this]() -> Eigen::Vector3d {
                                                             const Eigen::Vector3d p = posW().translation();
                                                             LOG_INFO("p: " << p.transpose());
                                                             Eigen::Vector3d end = p + velW().linear();
                                                             return end;
                                                           }));
}
void FloatingBasePosVelObserver::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  FloatingBasePosObserver::removeFromGUI(gui);
  gui.removeCategory({"Observers", name()});
}

} // namespace mc_observers

EXPORT_OBSERVER_MODULE("FloatingBasePosVel", mc_observers::FloatingBasePosVelObserver)
