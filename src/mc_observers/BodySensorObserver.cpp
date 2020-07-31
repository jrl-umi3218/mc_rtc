/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_observers/BodySensorObserver.h>

#include <mc_observers/ObserverMacros.h>

#include <mc_control/MCController.h>

#include <mc_rtc/gui/Arrow.h>

namespace mc_observers
{

void BodySensorObserver::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  robot_ = config("robot", ctl.robot().name());
  auto updateConfig = config("method", std::string{});
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

  fbSensorName_ = config("bodySensor", std::string("FloatingBase"));
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
    sva::MotionVecd sensorAcc(Eigen::Vector3d::Zero(), sensor.acceleration());
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

void BodySensorObserver::addToLogger(mc_control::MCController & ctl, std::string category)
{
  auto & logger = ctl.logger();
  category += "_" + name();
  Observer::addToLogger(ctl, category);
  logger.addLogEntry(category + "_posW", [this]() { return posW_; });
  logger.addLogEntry(category + "_velW", [this]() { return velW_; });
  logger.addLogEntry(category + "_accW", [this]() { return accW_; });
}
void BodySensorObserver::removeFromLogger(mc_control::MCController & ctl, std::string category)
{
  auto & logger = ctl.logger();
  category += "_" + name();
  Observer::removeFromLogger(ctl, category);
  logger.removeLogEntry("observer_" + name() + "_posW");
  logger.removeLogEntry("observer_" + name() + "_velW");
  logger.removeLogEntry("observer_" + name() + "_accW");
}

void BodySensorObserver::addToGUI(mc_control::MCController & ctl, std::vector<std::string> category)
{
  category.push_back(name());
  ctl.gui()->addElement(category,
                        mc_rtc::gui::Arrow("Velocity", mc_rtc::gui::ArrowConfig(mc_rtc::gui::Color{1., 0., 0.}),
                                           [this]() -> const Eigen::Vector3d & { return posW_.translation(); },
                                           [this]() -> Eigen::Vector3d {
                                             Eigen::Vector3d end = posW_.translation() + velW_.linear();
                                             return end;
                                           }));
}

} // namespace mc_observers

EXPORT_OBSERVER_MODULE("BodySensor", mc_observers::BodySensorObserver)
