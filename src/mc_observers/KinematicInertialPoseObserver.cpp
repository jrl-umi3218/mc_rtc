/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is  inspired by Stephane's Caron implementation as part of
 * lipm_walking_controller <https://github.com/stephane-caron/lipm_walking_controller>
 */

#include "KinematicInertialPoseObserver.h"

#include <mc_control/mc_controller.h>
#include <mc_rbdyn/rpy_utils.h>

namespace mc_observers
{
KinematicInertialPoseObserver::KinematicInertialPoseObserver(const std::string & name,
                                                             double dt,
                                                             const mc_rtc::Configuration & /* config */)
: Observer(name, dt), orientation_(Eigen::Matrix3d::Identity()), position_(Eigen::Vector3d::Zero()), leftFootRatio_(0.5)
{
}

void KinematicInertialPoseObserver::reset(const mc_control::MCController & ctl)
{
  run(ctl);
}

bool KinematicInertialPoseObserver::run(const mc_control::MCController & ctl)
{
  estimateOrientation(ctl.robot(), ctl.realRobot());
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
  sva::PTransformd X_0_rIMU = realRobot.bodyPosW(realRobot.bodySensor().parentBody());
  sva::PTransformd X_rIMU_rBase = X_0_rBase * X_0_rIMU.inv();
  Eigen::Matrix3d E_0_mIMU = robot.bodySensor().orientation().toRotationMatrix();
  Eigen::Matrix3d E_0_cBase = robot.posW().rotation();
  Eigen::Matrix3d E_0_mBase = X_rIMU_rBase.rotation() * E_0_mIMU;
  Eigen::Vector3d cRPY = mc_rbdyn::rpyFromMat(E_0_cBase);
  Eigen::Vector3d mRPY = mc_rbdyn::rpyFromMat(E_0_mBase);
  orientation_ = mc_rbdyn::rpyToMat(mRPY(0), mRPY(1), cRPY(2));
}

void KinematicInertialPoseObserver::estimatePosition(const mc_control::MCController & ctl)
{
  const auto & robot = ctl.robot();
  const auto & realRobot = ctl.realRobot();
  sva::PTransformd X_0_c = ctl.anchorFrame(robot);
  sva::PTransformd X_0_s = ctl.anchorFrame(realRobot);
  const sva::PTransformd & X_0_real = realRobot.posW();
  sva::PTransformd X_real_s = X_0_s * X_0_real.inv();
  const Eigen::Vector3d & r_c_0 = X_0_c.translation();
  const Eigen::Vector3d & r_s_real = X_real_s.translation();
  position_ = r_c_0 - orientation_.transpose() * r_s_real;
}

void KinematicInertialPoseObserver::updateRobots(const mc_control::MCController & /* ctl */,
                                                 mc_rbdyn::Robots & realRobots)
{
  realRobots.robot().posW(sva::PTransformd{orientation_, position_});
}

void KinematicInertialPoseObserver::addToLogger(const mc_control::MCController &, mc_rtc::Logger & logger)
{
  logger.addLogEntry("observer_" + name() + "_posW", [this]() { return posW(); });
}
void KinematicInertialPoseObserver::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntry("observer_" + name() + "_posW");
}

} // namespace mc_observers

EXPORT_OBSERVER_MODULE("KinematicInertialPose", mc_observers::KinematicInertialPoseObserver)
