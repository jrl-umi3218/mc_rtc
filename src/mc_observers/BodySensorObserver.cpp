/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_observers/BodySensorObserver.h>

#include <mc_observers/ObserverMacros.h>

#include <mc_control/MCController.h>

#include <mc_rtc/gui/Arrow.h>
#include <mc_rtc/gui/Transform.h>

namespace mc_observers
{

void BodySensorObserver::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  robot_ = config("robot", ctl.robot().name());
  updateRobot_ = config("updateRobot", static_cast<std::string>(robot_));
  fbSensorName_ = config("bodySensor", ctl.robot(robot_).bodySensor().name());
  if(!ctl.robots().hasRobot(robot_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] No robot named {}", name(), robot_);
  }
  if(!ctl.robots().hasRobot(updateRobot_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] No robot named {}", name(), updateRobot_);
  }
  if(updateFrom_ == Update::Sensor && !ctl.robot(robot_).hasBodySensor(fbSensorName_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] No sensor named {} in robot {}", name(), fbSensorName_,
                                                     robot_);
  }
  auto updateConfig = config("method", std::string{"sensor"});
  if(!updateConfig.empty())
  {
    if(updateConfig == "sensor")
    {
      updateFrom_ = Update::Sensor;
    }
    else
    {
      updateFrom_ = Update::Control;
    }
  }

  config("updatePose", updatePose_);
  config("updateVel", updateVel_);
  if(ctl.realRobot(updateRobot_).mb().joint(0).type() == rbd::Joint::Type::Fixed)
  {
    mc_rtc::log::warning("[{}] Requested update of the base velocity for robot {} but this robot has a fixed base, "
                         "velocity update will be ignored",
                         name(), updateRobot_);
    updateVel_ = false;
  }

  if(config.has("log"))
  {
    auto lConfig = config("log");
    lConfig("pose", logPos_);
    lConfig("velocity", logVel_);
    lConfig("acceleration", logAcc_);
  }

  if(config.has("gui"))
  {
    auto gConfig = config("gui");
    gConfig("pose", guiPos_);
    gConfig("velocity", guiVel_);
    gConfig("acceleration", guiAcc_);
    guiVelConfig_ = gConfig("velocityArrow", mc_rtc::gui::ArrowConfig(mc_rtc::gui::Color{1., 0., 0.}));
    guiAccConfig_ = gConfig("accelerationArrow", mc_rtc::gui::ArrowConfig(mc_rtc::gui::Color{1., 1., 0.}));
    gConfig("advanced", advancedGUI_);
  }

  desc_ = name_ + " (sensor=" + fbSensorName_ + ", update=" + updateConfig + ")";
}

void BodySensorObserver::reset(const mc_control::MCController & ctl)
{
  run(ctl);
}

bool BodySensorObserver::run(const mc_control::MCController & ctl)
{
  const auto & robot = ctl.robot(robot_);
  const auto & realRobot = ctl.realRobot(robot_);
  if(updateFrom_ == Update::Sensor)
  {
    // Update free flyer from body sensor
    // Note that if the body to which the sensor is attached is not the
    // floating base, the kinematic transformation between that body and the
    // floating base is used to obtain the floating base pose.
    // It is assumed here that the floating base sensor and encoders are
    // synchronized.
    const auto & sensor = robot.bodySensor(fbSensorName_);
    const auto & fb = realRobot.mb().body(0).name();
    sva::PTransformd X_0_s(sensor.orientation(), sensor.position());

    const auto X_s_b = sensor.X_b_s().inv();
    sva::PTransformd X_b_fb = realRobot.X_b1_b2(sensor.parentBody(), fb);
    sva::PTransformd X_s_fb = X_b_fb * X_s_b;
    // Note 1: This renormalizes and reorthogonalizes the rotation matrix to
    // avoid numerical errors slowly adding up over the course of many
    // iterations. It would however be better to compute the relative pose
    // X_b_fb withoug passing through the world frame, which would not be
    // subject from this denormalization issue.
    // Note 2: This issue only occurs for fixed-base robots, as the posW()
    // performs the quaternion conversion and normalization itself
    {
      Eigen::Quaterniond rot(X_s_fb.rotation());
      rot.normalize();
      X_s_fb.rotation() = rot.toRotationMatrix();
    }
    posW_ = X_s_fb * X_0_s;

    sva::MotionVecd sensorVel(sensor.angularVelocity(), sensor.linearVelocity());
    velW_ = X_s_fb * sensorVel;
    sva::MotionVecd sensorAcc(sensor.angularAcceleration(), sensor.linearAcceleration());
    accW_ = X_s_fb * sensorAcc;
  }
  else /* if(updateFrom_ == Update::Control) */
  {
    posW_ = robot.posW();
    velW_ = robot.velW();
    accW_ = robot.accW();
  }
  return true;
}

void BodySensorObserver::update(mc_control::MCController & ctl)
{
  auto & realRobot = ctl.realRobots().robot(updateRobot_);
  if(updatePose_)
  {
    realRobot.posW(posW_);
  }
  if(updateVel_)
  {
    realRobot.velW(velW_);
  }
}

void BodySensorObserver::addToLogger(const mc_control::MCController &,
                                     mc_rtc::Logger & logger,
                                     const std::string & category)
{
  auto cat = category + "_" + fbSensorName_;
  if(logPos_)
  {
    logger.addLogEntry(cat + "_posW", [this]() { return posW_; });
  }
  if(logVel_)
  {
    logger.addLogEntry(cat + "_velW", [this]() { return velW_; });
  }
  if(logAcc_)
  {
    logger.addLogEntry(cat + "_accW", [this]() { return accW_; });
  }
}

void BodySensorObserver::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  auto cat = category + "_" + fbSensorName_;
  logger.removeLogEntry(cat + "_posW");
  logger.removeLogEntry(cat + "_velW");
  logger.removeLogEntry(cat + "_accW");
}

void BodySensorObserver::addToGUI(const mc_control::MCController & ctl,
                                  mc_rtc::gui::StateBuilder & gui,
                                  const std::vector<std::string> & category)
{
  auto showPose = [this, &gui, category]() {
    if(guiPos_)
    {
      gui.addElement(category, mc_rtc::gui::Transform("Pose", [this]() -> const sva::PTransformd & { return posW_; }));
    }
    else
    {
      gui.removeElement(category, "Pose");
    }
  };
  auto showVel = [this, &gui, category]() {
    if(guiVel_)
    {
      gui.addElement(category, mc_rtc::gui::Arrow("Velocity", guiVelConfig_,
                                                  [this]() -> const Eigen::Vector3d & { return posW_.translation(); },
                                                  [this]() -> Eigen::Vector3d {
                                                    Eigen::Vector3d end = posW_.translation() + velW_.linear();
                                                    return end;
                                                  }));
    }
    else
    {
      gui.removeElement(category, "Velocity");
    }
  };
  auto showAcc = [this, &gui, category]() {
    if(guiAcc_)
    {
      gui.addElement(category, mc_rtc::gui::Arrow("Acceleration", guiAccConfig_,
                                                  [this]() -> const Eigen::Vector3d & { return posW_.translation(); },
                                                  [this]() -> Eigen::Vector3d {
                                                    Eigen::Vector3d end = posW_.translation() + accW_.linear();
                                                    return end;
                                                  }));
    }
    else
    {
      gui.removeElement(category, "Acceleration");
    }
  };

  gui.addElement(category,
                 mc_rtc::gui::Checkbox("Show pose", [this]() { return guiPos_; },
                                       [this, showPose]() {
                                         guiPos_ = !guiPos_;
                                         showPose();
                                       }),
                 mc_rtc::gui::Checkbox("Show velocity", [this]() { return guiVel_; },
                                       [this, showVel]() {
                                         guiVel_ = !guiVel_;
                                         showVel();
                                       }),
                 mc_rtc::gui::Checkbox("Show acceleration", [this]() { return guiAcc_; },
                                       [this, showAcc]() {
                                         guiAcc_ = !guiAcc_;
                                         showAcc();
                                       }));

  showPose();
  showVel();
  showAcc();

  if(advancedGUI_)
  {
    auto sensorNames = std::vector<std::string>{};
    auto & bodySensors = ctl.robot(robot_).bodySensors();
    sensorNames.resize(bodySensors.size());
    std::transform(bodySensors.begin(), bodySensors.end(), sensorNames.begin(),
                   [](const mc_rbdyn::BodySensor & bs) { return bs.name(); });

    auto advancedCat = category;
    advancedCat.push_back("Advanced");
    gui.addElement(advancedCat,
                   mc_rtc::gui::ComboInput("BodySensor", sensorNames, [this]() { return fbSensorName_; },
                                           [this](const std::string & sensor) { fbSensorName_ = sensor; }));
  }
}

} // namespace mc_observers

EXPORT_OBSERVER_MODULE("BodySensor", mc_observers::BodySensorObserver)
