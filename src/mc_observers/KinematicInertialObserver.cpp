/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "KinematicInertialObserver.h"

#include <mc_control/mc_controller.h>

namespace mc_observers
{
KinematicInertialObserver::KinematicInertialObserver(const std::string & name,
                                                     double dt,
                                                     const mc_rtc::Configuration & config)
: KinematicInertialPoseObserver(name, dt, config), velFilter_(dt, /* cutoff period = */ 0.01)
{
  desc_ = name_ + " (cutoff=" + std::to_string(velFilter_.cutoffPeriod()) + ")";
}

void KinematicInertialObserver::reset(const mc_control::MCController & ctl)
{
  reset(ctl, ctl.realRobot().velW());
}

void KinematicInertialObserver::reset(const mc_control::MCController & ctl, const sva::MotionVecd & velW)
{
  KinematicInertialPoseObserver::reset(ctl);
  posWPrev_ = KinematicInertialPoseObserver::posW();
  velW_ = velW;
  velFilter_.reset(velW);
}

bool KinematicInertialObserver::run(const mc_control::MCController & ctl)
{
  KinematicInertialPoseObserver::run(ctl);
  const sva::PTransformd posW = KinematicInertialPoseObserver::posW();
  sva::MotionVecd errVel = sva::transformError(posWPrev_, posW) / dt();
  velFilter_.update(errVel);
  velW_ = velFilter_.vel();
  velWfd_ = errVel;
  posWPrev_ = posW;
  return true;
}

void KinematicInertialObserver::updateRobots(const mc_control::MCController & ctl, mc_rbdyn::Robots & realRobots)
{
  KinematicInertialPoseObserver::updateRobots(ctl, realRobots);
  realRobots.robot().velW(velW_);
}

void KinematicInertialObserver::updateBodySensor(mc_rbdyn::Robots & realRobots, const std::string & sensorName)
{
  KinematicInertialPoseObserver::updateBodySensor(realRobots, sensorName);
  auto & sensor = realRobots.robot().bodySensor(sensorName);
  sensor.linearVelocity(velW_.linear());
  sensor.angularVelocity(velW_.angular());
}

const sva::MotionVecd & KinematicInertialObserver::velW() const
{
  return velW_;
}

void KinematicInertialObserver::addToLogger(const mc_control::MCController & ctl, mc_rtc::Logger & logger)
{
  KinematicInertialPoseObserver::addToLogger(ctl, logger);
  logger.addLogEntry("observer_" + name() + "_velW", [this]() { return velW_; });
  logger.addLogEntry("observer_" + name() + "_velWfd", [this]() { return velWfd_; });
}
void KinematicInertialObserver::removeFromLogger(mc_rtc::Logger & logger)
{
  KinematicInertialPoseObserver::removeFromLogger(logger);
  logger.removeLogEntry("observer_" + name() + "_velW");
  logger.removeLogEntry("observer_" + name() + "_velWfd");
}
void KinematicInertialObserver::addToGUI(const mc_control::MCController & ctl, mc_rtc::gui::StateBuilder & gui)
{
  KinematicInertialPoseObserver::addToGUI(ctl, gui);
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
