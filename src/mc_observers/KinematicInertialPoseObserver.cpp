/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is  inspired by Stephane's Caron implementation as part of
 * lipm_walking_controller <https://github.com/stephane-caron/lipm_walking_controller>
 */

#include <mc_observers/KinematicInertialPoseObserver.h>
#include <mc_observers/ObserverMacros.h>

#include <mc_control/MCController.h>
#include <mc_rbdyn/rpy_utils.h>

namespace mc_observers
{

void KinematicInertialPoseObserver::configure(const mc_control::MCController & ctl,
                                              const mc_rtc::Configuration & config)
{
  config("showAnchorFrame", showAnchorFrame_);
  robot_ = config("robot", ctl.robot().name());
  imuSensor_ = config("imuBodySensor", ctl.robot().bodySensor().name());
}

void KinematicInertialPoseObserver::reset(const mc_control::MCController & ctl)
{
  run(ctl);
}

bool KinematicInertialPoseObserver::run(const mc_control::MCController & ctl)
{
  estimateOrientation(ctl.robot(robot_), ctl.realRobot(robot_));
  estimatePosition(ctl);
  return true;
}

void KinematicInertialPoseObserver::estimateOrientation(const mc_rbdyn::Robot & robot,
                                                        const mc_rbdyn::Robot & realRobot)
{
  // Prefixes:
  // c for control-robot model
  // r for real-robot model
  // m for estimated/measured quantities
  sva::PTransformd X_0_rBase = realRobot.posW();
  const auto & imuSensor = realRobot.bodySensor(imuSensor_);
  sva::PTransformd X_0_rIMU = realRobot.bodyPosW(imuSensor.parentBody());
  sva::PTransformd X_rIMU_rBase = X_0_rBase * X_0_rIMU.inv();
  Eigen::Matrix3d E_0_mIMU = imuSensor.orientation().toRotationMatrix();
  Eigen::Matrix3d E_0_cBase = robot.posW().rotation();
  Eigen::Matrix3d E_0_mBase = X_rIMU_rBase.rotation() * E_0_mIMU;
  Eigen::Vector3d cRPY = mc_rbdyn::rpyFromMat(E_0_cBase);
  Eigen::Vector3d mRPY = mc_rbdyn::rpyFromMat(E_0_mBase);
  orientation_ = mc_rbdyn::rpyToMat(mRPY(0), mRPY(1), cRPY(2));
}

void KinematicInertialPoseObserver::estimatePosition(const mc_control::MCController & ctl)
{
  // FIXME not multi-robot. Use datastore instead
  const sva::PTransformd & X_0_c = ctl.anchorFrame();
  const sva::PTransformd & X_0_s = ctl.anchorFrameReal();
  const sva::PTransformd X_real_s = X_0_s * ctl.realRobot(robot_).posW().inv();
  const Eigen::Vector3d & r_c_0 = X_0_c.translation();
  const Eigen::Vector3d & r_s_real = X_real_s.translation();
  position_ = r_c_0 - orientation_.transpose() * r_s_real;
}

void KinematicInertialPoseObserver::updateRobots(mc_control::MCController & ctl)
{
  auto & robot = ctl.realRobot(robot_);
  robot.posW(sva::PTransformd{orientation_, position_});
}

void KinematicInertialPoseObserver::addToLogger(mc_control::MCController & ctl, const std::string & category)
{
  auto & logger = ctl.logger();
  logger.addLogEntry(category + "_posW", [this]() { return posW(); });
  logger.addLogEntry(category + "_anchorFrame", [&ctl]() { return ctl.anchorFrame(); });
  logger.addLogEntry(category + "_anchorFrameReal", [&ctl]() { return ctl.anchorFrameReal(); });
}

void KinematicInertialPoseObserver::removeFromLogger(mc_control::MCController & ctl, const std::string & category)
{
  auto & logger = ctl.logger();
  logger.removeLogEntry(category + "_posW");
  logger.removeLogEntry(category + "_anchorFrame");
  logger.removeLogEntry(category + "_anchorFrameReal");
}

void KinematicInertialPoseObserver::addToGUI(mc_control::MCController & ctl, const std::vector<std::string> & category)
{
  if(showAnchorFrame_)
  {
    ctl.gui()->addElement(category,
                          mc_rtc::gui::Transform("anchorFrameControl", [&ctl]() { return ctl.anchorFrame(); }),
                          mc_rtc::gui::Transform("anchorFrameReal", [&ctl]() { return ctl.anchorFrameReal(); }));
  }
}

} // namespace mc_observers

EXPORT_OBSERVER_MODULE("KinematicInertialPose", mc_observers::KinematicInertialPoseObserver)
