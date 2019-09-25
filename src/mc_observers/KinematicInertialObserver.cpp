/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "KinematicInertialObserver.h"

namespace mc_observers
{
KinematicInertialObserver::KinematicInertialObserver(const std::string & name,
                                                     double dt,
                                                     const mc_rtc::Configuration & config)
: KinematicInertialPoseObserver(name, dt, config), velFilter_(dt, /* cutoff period = */ 0.01)
{
  LOG_SUCCESS("KinematicInertialObserver created");
}

void KinematicInertialObserver::reset(const mc_rbdyn::Robot & realRobot)
{
  reset(realRobot, realRobot.velW());
}

void KinematicInertialObserver::reset(const mc_rbdyn::Robot & realRobot, const sva::MotionVecd & velW)
{
  KinematicInertialPoseObserver::reset(realRobot);
  posWPrev_ = KinematicInertialPoseObserver::posW();
  velW_ = velW;
  velFilter_.reset(velW);
}

bool KinematicInertialObserver::run(const mc_rbdyn::Robot & realRobot)
{
  KinematicInertialPoseObserver::run(realRobot);
  const sva::PTransformd posW = KinematicInertialPoseObserver::posW();
  sva::MotionVecd errVel = sva::transformError(posWPrev_, posW) / dt();
  velFilter_.update(errVel);
  velW_ = velFilter_.vel();
  posWPrev_ = posW;
  return true;
}

void KinematicInertialObserver::updateRobot(mc_rbdyn::Robot & realRobot)
{
  KinematicInertialPoseObserver::updateRobot(realRobot);
  realRobot.velW(velW_);
}

void KinematicInertialObserver::updateBodySensor(mc_rbdyn::Robot & realRobot, const std::string & sensorName)
{
  KinematicInertialPoseObserver::updateBodySensor(realRobot, sensorName);
  auto & sensor = realRobot.bodySensor(sensorName);
  sensor.linearVelocity(velW_.linear());
  sensor.angularVelocity(velW_.angular());
}

const sva::MotionVecd & KinematicInertialObserver::velW() const
{
  return velW_;
}

void KinematicInertialObserver::addToLogger(mc_rtc::Logger & logger)
{
  KinematicInertialPoseObserver::addToLogger(logger);
  logger.addLogEntry("observer_" + name() + "_velW", [this]() { return velW_; });
}
void KinematicInertialObserver::removeFromLogger(mc_rtc::Logger & logger)
{
  KinematicInertialPoseObserver::removeFromLogger(logger);
  logger.removeLogEntry("observer_" + name() + "_velW");
}
void KinematicInertialObserver::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  KinematicInertialPoseObserver::addToGUI(gui);
  gui.addElement({"Observers", name()}, mc_rtc::gui::Arrow("Velocity", [this]() { return posW().translation(); },
                                                           [this]() -> Eigen::Vector3d {
                                                             const Eigen::Vector3d p = posW().translation();
                                                             Eigen::Vector3d end = p + velW().linear();
                                                             return end;
                                                           }));
}
void KinematicInertialObserver::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  KinematicInertialPoseObserver::removeFromGUI(gui);
  gui.removeCategory({"Observers", name()});
}

} // namespace mc_observers

EXPORT_OBSERVER_MODULE("KinematicInertial", mc_observers::KinematicInertialObserver)
