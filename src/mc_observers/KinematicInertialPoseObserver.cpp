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
  robot_ = config("robot", ctl.robot().name());
  realRobot_ = config("realRobot", ctl.realRobot().name());
  imuSensor_ = config("imuBodySensor", ctl.robot().bodySensor().name());
  config("anchorFrame", anchorFrameFunction_);
  config("showAnchorFrame", showAnchorFrame_);
  config("showAnchorFrameReal", showAnchorFrameReal_);
  config("showPose", showPose_);
}

void KinematicInertialPoseObserver::reset(const mc_control::MCController & ctl)
{
  run(ctl);
}

bool KinematicInertialPoseObserver::run(const mc_control::MCController & ctl)
{
  if(!ctl.datastore().has("Observer::anchorFrame"))
  {
    error_ =
        fmt::format("Observer {} requires an \"Observer::anchorFrame\" function in the datastore.\n\tPlease refer to "
                    "https://jrl-umi3218.github.io/mc_rtc/tutorials/recipes/observers.html for further details.",
                    name());
    return false;
  }
  X_0_anchorFrame_ = ctl.datastore().call<sva::PTransformd>(anchorFrameFunction_, ctl.robot(robot_));
  X_0_anchorFrameReal_ = ctl.datastore().call<sva::PTransformd>(anchorFrameFunction_, ctl.realRobot(realRobot_));
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
  pose_.rotation() = mc_rbdyn::rpyToMat(mRPY(0), mRPY(1), cRPY(2));
}

void KinematicInertialPoseObserver::estimatePosition(const mc_control::MCController & ctl)
{
  const sva::PTransformd X_real_s = X_0_anchorFrameReal_ * ctl.realRobot(robot_).posW().inv();
  const Eigen::Vector3d & r_c_0 = X_0_anchorFrame_.translation();
  const Eigen::Vector3d & r_s_real = X_real_s.translation();
  pose_.translation() = r_c_0 - pose_.rotation().transpose() * r_s_real;
}

void KinematicInertialPoseObserver::updateRobots(mc_control::MCController & ctl)
{
  auto & robot = ctl.realRobot(robot_);
  robot.posW(pose_);
}

void KinematicInertialPoseObserver::addToLogger(mc_control::MCController & ctl, const std::string & category)
{
  auto & logger = ctl.logger();
  logger.addLogEntry(category + "_posW", [this]() -> const sva::PTransformd & { return pose_; });
  logger.addLogEntry(category + "_anchorFrame", [this]() -> const sva::PTransformd & { return X_0_anchorFrame_; });
  logger.addLogEntry(category + "_anchorFrameReal",
                     [this]() -> const sva::PTransformd & { return X_0_anchorFrameReal_; });
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
  auto showHideAnchorFrame = [&ctl, category](const std::string & name, bool show,
                                              const sva::PTransformd & anchorFrame) {
    auto cat = category;
    cat.push_back("Markers");
    ctl.gui()->removeElement(cat, name);
    if(show)
    {
      ctl.gui()->addElement(
          cat, mc_rtc::gui::Transform(name, [&anchorFrame]() -> const sva::PTransformd & { return anchorFrame; }));
    }
  };
  auto showHidePose = [this, category, &ctl]() {
    std::string name = "Pose";
    auto cat = category;
    cat.push_back("Markers");
    ctl.gui()->removeElement(cat, name);
    if(showPose_)
    {
      ctl.gui()->addElement(cat, mc_rtc::gui::Transform(name, [this]() -> const sva::PTransformd & { return pose_; }));
    }
  };

  ctl.gui()->addElement(
      category,
      mc_rtc::gui::Checkbox("Show anchor frame (control)", [this]() { return showAnchorFrame_; },
                            [this, showHideAnchorFrame]() {
                              showAnchorFrame_ = !showAnchorFrame_;
                              showHideAnchorFrame("Anchor Frame (control)", showAnchorFrame_, X_0_anchorFrame_);
                            }),
      mc_rtc::gui::Checkbox("Show anchor frame (real)", [this]() { return showAnchorFrameReal_; },
                            [this, showHideAnchorFrame]() {
                              showAnchorFrameReal_ = !showAnchorFrameReal_;
                              showHideAnchorFrame("Anchor Frame (real)", showAnchorFrameReal_, X_0_anchorFrameReal_);
                            }),
      mc_rtc::gui::Checkbox("Show pose", [this]() { return showPose_; },
                            [this, showHidePose]() {
                              showPose_ = !showPose_;
                              showHidePose();
                            }));

  showHideAnchorFrame("Anchor Frame (control)", showAnchorFrame_, X_0_anchorFrame_);
  showHideAnchorFrame("Anchor Frame (real)", showAnchorFrameReal_, X_0_anchorFrameReal_);
  showHidePose();
}

} // namespace mc_observers

EXPORT_OBSERVER_MODULE("KinematicInertialPose", mc_observers::KinematicInertialPoseObserver)
