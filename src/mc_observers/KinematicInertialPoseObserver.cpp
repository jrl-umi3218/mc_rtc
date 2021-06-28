/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
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
  realRobot_ = config("updateRobot", ctl.realRobot().name());
  imuSensor_ = config("imuBodySensor", ctl.robot().bodySensor().name());
  anchorFrameFunction_ = "KinematicAnchorFrame::" + ctl.robot(robot_).name();
  if(config.has("anchorFrame"))
  {
    auto afConfig = config("anchorFrame");
    afConfig("datastoreFunction", anchorFrameFunction_);
    afConfig("maxAnchorFrameDiscontinuity", maxAnchorFrameDiscontinuity_);
  }
  if(config.has("gui"))
  {
    auto gConfig = config("gui");
    gConfig("anchorFrame", showAnchorFrame_);
    gConfig("anchorFrameReal", showAnchorFrameReal_);
    gConfig("pose", showPose_);
    gConfig("advanced", advancedGUI_);
  }
  if(config.has("log"))
  {
    auto lConfig = config("log");
    lConfig("pose", logPose_);
    lConfig("anchorFrame", logAnchorFrame_);
  }
}

void KinematicInertialPoseObserver::reset(const mc_control::MCController & ctl)
{
  pose_ = ctl.realRobot(robot_).posW();
  firstIter_ = true;
}

bool KinematicInertialPoseObserver::run(const mc_control::MCController & ctl)
{
  if(!ctl.datastore().has(anchorFrameFunction_))
  {
    error_ = fmt::format(
        "Observer {} requires a \"{}\" function in the datastore to provide the observer's kinematic anchor frame.\n"
        "Please refer to https://jrl-umi3218.github.io/mc_rtc/tutorials/recipes/observers.html for further details.",
        name(), anchorFrameFunction_);
    return false;
  }
  anchorFrameJumped_ = false;
  auto anchorFrame = ctl.datastore().call<sva::PTransformd>(anchorFrameFunction_, ctl.robot(robot_));
  auto anchorFrameReal = ctl.datastore().call<sva::PTransformd>(anchorFrameFunction_, ctl.realRobot(realRobot_));
  auto error = (anchorFrame.translation() - X_0_anchorFrame_.translation()).norm();
  if(firstIter_)
  { // Ignore anchor frame check on first iteration
    firstIter_ = false;
  }
  else
  { // Check whether anchor frame jumped
    if(error > maxAnchorFrameDiscontinuity_)
    {
      mc_rtc::log::warning("[{}] Control anchor frame jumped from [{}] to [{}] (error norm {} > threshold {})", name(),
                           X_0_anchorFrame_.translation().transpose(), anchorFrame.translation().transpose(), error,
                           maxAnchorFrameDiscontinuity_);
      anchorFrameJumped_ = true;
    }
    if((anchorFrameReal.translation() - X_0_anchorFrameReal_.translation()).norm() > maxAnchorFrameDiscontinuity_)
    {
      mc_rtc::log::warning("[{}] Real anchor frame jumped from [{}] to [{}] (error norm {:.3f} > threshold {:.3f})",
                           name(), X_0_anchorFrameReal_.translation().transpose(),
                           anchorFrameReal.translation().transpose(), error, maxAnchorFrameDiscontinuity_);
      anchorFrameJumped_ = true;
    }
  }
  X_0_anchorFrame_ = anchorFrame;
  X_0_anchorFrameReal_ = anchorFrameReal;
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
  const sva::PTransformd & X_0_rBase = realRobot.posW();
  const auto & imuSensor = realRobot.bodySensor(imuSensor_);
  const auto & imuParent = imuSensor.parentBody();
  const sva::PTransformd & X_0_rIMU = realRobot.bodyPosW(imuParent);
  const sva::PTransformd & X_0_cIMU = robot.bodyPosW(imuParent);
  sva::PTransformd X_rIMU_rBase = X_0_rBase * X_0_rIMU.inv();

  Eigen::Matrix3d E_0_mIMU = imuSensor.orientation().toRotationMatrix();
  const Eigen::Matrix3d & E_0_cIMU = X_0_cIMU.rotation();
  // Estimate IMU orientation: merges roll+pitch from measurement with yaw from control
  Eigen::Matrix3d E_0_eIMU = mergeRoll1Pitch1WithYaw2(E_0_mIMU, E_0_cIMU);
  pose_.rotation() = E_0_eIMU.transpose() * X_rIMU_rBase.rotation();
}

void KinematicInertialPoseObserver::estimatePosition(const mc_control::MCController & ctl)
{
  // Relative transformation between the real anchor frame and the floating base
  // Uses the FK with the real robot encoder measurements
  const sva::PTransformd X_real_s = X_0_anchorFrameReal_ * ctl.realRobot(robot_).posW().inv();
  // Position of the control anchor frame (world)
  const Eigen::Vector3d & r_c_0 = X_0_anchorFrame_.translation();
  // Position of the real anchor frame (world)
  const Eigen::Vector3d & r_s_real = X_real_s.translation();
  // The floating base position is estimated by applying the kinematics
  // transformation between the real robot's anchor frame and its floating base
  // (kinematics only), rotated by the estimated rotation of the real floating
  // base from the IMU (see estimateOrientation)
  pose_.translation() = r_c_0 - pose_.rotation().transpose() * r_s_real;
}

void KinematicInertialPoseObserver::update(mc_control::MCController & ctl)
{
  auto & robot = ctl.realRobot(robot_);
  robot.posW(pose_);
}

void KinematicInertialPoseObserver::addToLogger(const mc_control::MCController &,
                                                mc_rtc::Logger & logger,
                                                const std::string & category)
{
  if(logPose_)
  {
    MC_RTC_LOG_HELPER(category + "_posW", pose_);
  }
  if(logAnchorFrame_)
  {
    MC_RTC_LOG_HELPER(category + "_anchorFrame", X_0_anchorFrame_);
    MC_RTC_LOG_HELPER(category + "_anchorFrameReal", X_0_anchorFrameReal_);
  }
}

void KinematicInertialPoseObserver::addToGUI(const mc_control::MCController & ctl,
                                             mc_rtc::gui::StateBuilder & gui,
                                             const std::vector<std::string> & category)
{
  auto showHideAnchorFrame = [&gui, category](const std::string & name, bool show,
                                              const sva::PTransformd & anchorFrame) {
    auto cat = category;
    cat.push_back("Markers");
    gui.removeElement(cat, name);
    if(show)
    {
      gui.addElement(
          cat, mc_rtc::gui::Transform(name, [&anchorFrame]() -> const sva::PTransformd & { return anchorFrame; }));
    }
  };
  auto showHidePose = [this, category, &gui]() {
    std::string name = "Pose";
    auto cat = category;
    cat.push_back("Markers");
    gui.removeElement(cat, name);
    if(showPose_)
    {
      gui.addElement(cat, mc_rtc::gui::Transform(name, [this]() -> const sva::PTransformd & { return pose_; }));
    }
  };

  gui.addElement(category,
                 mc_rtc::gui::Checkbox(
                     "Show anchor frame (control)", [this]() { return showAnchorFrame_; },
                     [this, showHideAnchorFrame]() {
                       showAnchorFrame_ = !showAnchorFrame_;
                       showHideAnchorFrame("Anchor Frame (control)", showAnchorFrame_, X_0_anchorFrame_);
                     }),
                 mc_rtc::gui::Checkbox(
                     "Show anchor frame (real)", [this]() { return showAnchorFrameReal_; },
                     [this, showHideAnchorFrame]() {
                       showAnchorFrameReal_ = !showAnchorFrameReal_;
                       showHideAnchorFrame("Anchor Frame (real)", showAnchorFrameReal_, X_0_anchorFrameReal_);
                     }),
                 mc_rtc::gui::Checkbox(
                     "Show pose", [this]() { return showPose_; },
                     [this, showHidePose]() {
                       showPose_ = !showPose_;
                       showHidePose();
                     }));

  if(advancedGUI_)
  {
    auto sensorNames = std::vector<std::string>{};
    auto & bodySensors = ctl.robot(robot_).bodySensors();
    sensorNames.resize(bodySensors.size());
    std::transform(bodySensors.begin(), bodySensors.end(), sensorNames.begin(),
                   [](const mc_rbdyn::BodySensor & bs) { return bs.name(); });

    auto advancedCat = category;
    advancedCat.push_back("Advanced");
    gui.addElement(advancedCat, mc_rtc::gui::ComboInput(
                                    "BodySensor", sensorNames, [this]() { return imuSensor_; },
                                    [this](const std::string & sensor) { imuSensor_ = sensor; }));
  }

  showHideAnchorFrame("Anchor Frame (control)", showAnchorFrame_, X_0_anchorFrame_);
  showHideAnchorFrame("Anchor Frame (real)", showAnchorFrameReal_, X_0_anchorFrameReal_);
  showHidePose();
}

inline Eigen::Matrix3d KinematicInertialPoseObserver::mergeRoll1Pitch1WithYaw2(const Eigen::Matrix3d & R1,
                                                                               const Eigen::Matrix3d & R2,
                                                                               double epsilonAngle)
{
  using Matrix3 = Eigen::Matrix3d;
  using Vector3 = Eigen::Vector3d;

  const Vector3 & ez = Vector3::UnitZ();
  Matrix3 R_temp1, R_temp2;
  Vector3 v1 = R1 * ez;
  Vector3 mlxv1 = (R2 * Vector3::UnitX()).cross(v1);
  double n2 = mlxv1.squaredNorm();

  if(n2 > epsilonAngle * epsilonAngle)
  {
    // we take m=ex
    R_temp1 << -Vector3::UnitY(), Vector3::UnitX(), ez;
    mlxv1 /= sqrt(n2);
    R_temp2 << mlxv1.transpose(), v1.cross(mlxv1).transpose(), v1.transpose();
    return R_temp1 * R_temp2;
  }
  else
  {
    // we take m=ey
    mlxv1 = (R2 * Vector3::UnitY()).cross(v1).normalized();
    R_temp2 << mlxv1.transpose(), v1.cross(mlxv1).transpose(), v1.transpose();
    return R_temp2.transpose();
  }
}

} // namespace mc_observers

EXPORT_OBSERVER_MODULE("KinematicInertialPose", mc_observers::KinematicInertialPoseObserver)
