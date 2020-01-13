/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_observers/BodySensorObserver.h>

#include <mc_observers/ObserverMacros.h>

#include <mc_control/MCController.h>

#include <mc_rtc/gui/Arrow.h>

namespace mc_observers
{
BodySensorObserver::BodySensorObserver(const std::string & name, double dt, const mc_rtc::Configuration & config)
: Observer(name, dt, config)
{
  auto updateConfig = config("UpdateFrom", std::string{});
  if(!updateConfig.empty())
  {
    if(updateConfig == "estimator")
    {
      updateFrom_ = Update::Estimator;
    }
    else
    {
      updateFrom_ = Update::Control;
    }
  }

  fbSensorName_ = config("FloatingBaseSensor", std::string("FloatingBase"));
  desc_ = name_ + " (sensor=" + fbSensorName_ + ",update=" + updateConfig + ")";
}

void BodySensorObserver::reset(const mc_control::MCController & ctl)
{
  if(updateFrom_ == Update::Estimator)
  {
    if(!ctl.robot().hasBodySensor(fbSensorName_))
    {
      LOG_ERROR_AND_THROW(std::runtime_error, "[BodySensorObserver] Bodysensor "
                                                  << fbSensorName_ << " is requested but does not exist in robot "
                                                  << ctl.robot().name());
    }
  }
  run(ctl);
}

bool BodySensorObserver::run(const mc_control::MCController & ctl)
{
  const auto & robot = ctl.robot();
  const auto & realRobot = ctl.realRobot();
  if(updateFrom_ == Update::Estimator)
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
  }
  else /* if(updateFrom_ == Update::Control) */
  {
    posW_ = robot.posW();
    velW_ = robot.velW();
  }
  return true;
}

void BodySensorObserver::updateRobots(const mc_control::MCController & /* ctl */, mc_rbdyn::Robots & realRobots)
{
  realRobots.robot().posW(posW_);
  realRobots.robot().velW(velW_);
}

void BodySensorObserver::addToLogger(const mc_control::MCController & ctl, mc_rtc::Logger & logger)
{
  Observer::addToLogger(ctl, logger);
  logger.addLogEntry("observer_" + name() + "_posW", [this]() { return posW_; });
  logger.addLogEntry("observer_" + name() + "_velW", [this]() { return velW_; });
}
void BodySensorObserver::removeFromLogger(mc_rtc::Logger & logger)
{
  Observer::removeFromLogger(logger);
  logger.removeLogEntry("observer_" + name() + "_posW");
  logger.removeLogEntry("observer_" + name() + "_velW");
}

void BodySensorObserver::addToGUI(const mc_control::MCController &, mc_rtc::gui::StateBuilder & gui)
{
  gui.addElement({"Observers", name()},
                 mc_rtc::gui::Arrow("Velocity", mc_rtc::gui::ArrowConfig(mc_rtc::gui::Color{1., 0., 0.}),
                                    [this]() -> const Eigen::Vector3d & { return posW_.translation(); },
                                    [this]() -> Eigen::Vector3d {
                                      Eigen::Vector3d end = posW_.translation() + velW_.linear();
                                      return end;
                                    }));
}

} // namespace mc_observers

EXPORT_OBSERVER_MODULE("BodySensor", mc_observers::BodySensorObserver)
