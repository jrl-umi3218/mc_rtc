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
  fbSensorName_ = config("bodySensor", ctl.robot(robot_).bodySensor().name());
  if(!ctl.robots().hasRobot(robot_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] No robot named {}", name(), robot_);
  }
  if(!ctl.robot(robot_).hasBodySensor(fbSensorName_))
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
  }

  desc_ = name_ + " (sensor=" + fbSensorName_ + ", update=" + updateConfig + ")";
}

void BodySensorObserver::reset(const mc_control::MCController & ctl)
{
  if(updateFrom_ == Update::Sensor)
  {
    if(!ctl.robot().hasBodySensor(fbSensorName_))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "[BodySensorObserver] Bodysensor {} is requested but does not exist in robot {}", fbSensorName_,
          ctl.robot().name());
    }
    name_ += "_" + fbSensorName_;
  }
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

void BodySensorObserver::updateRobots(mc_control::MCController & ctl)
{
  auto & realRobot = ctl.realRobots().robot(robot_);
  realRobot.posW(posW_);
  realRobot.velW(velW_);
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

void BodySensorObserver::addToGUI(const mc_control::MCController &,
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
}

} // namespace mc_observers

EXPORT_OBSERVER_MODULE("BodySensor", mc_observers::BodySensorObserver)
