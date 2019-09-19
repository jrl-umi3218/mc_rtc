#include "BodySensorObserver.h"

namespace mc_observers
{
BodySensorObserver::BodySensorObserver(const std::string & name, double dt, const mc_rtc::Configuration & config)
: Observer(name, dt, config)
{
  LOG_INFO("BodySensorObserver config: \n" << config.dump(true));
  updateFbFromSensor_ = config("UpdateFloatingBaseFromSensor", true);
  updateFbFromControl_ = config("UpdateFloatingBaseFromControl", false);
  fbSensorName_ = config("FloatingBaseSensor", std::string("FloatingBase"));
}

void BodySensorObserver::reset(const mc_rbdyn::Robot & realRobot)
{
  const auto & bs = robot().bodySensor(fbSensorName_);
  posW_ = {bs.orientation(), bs.position()};
  velW_ = {bs.angularVelocity(), bs.linearVelocity()};
  LOG_INFO("[reset] BodySensorObserver bodysensor pos: " << bs.position());
}

bool BodySensorObserver::run(const mc_rbdyn::Robot & realRobot)
{
  if(updateFbFromControl_)
  {
    posW_ = robot().posW();
    velW_ = robot().velW();
  }
  if(updateFbFromSensor_)
  {
    // Update free flyer from body sensor
    // Note that if the body to which the sensor is attached is not the
    // floating base, the kinematic transformation between that body and the
    // floating base is used to obtain the floating base pose.
    // It is assumed here that the floating base sensor and encoders are
    // synchronized.
    const auto & sensor = robot().bodySensor(fbSensorName_);
    const auto & fb = robot().mb().body(0).name();
    sva::PTransformd X_0_s(sensor.orientation(), sensor.position());
    const auto & X_s_b = sensor.X_b_s().inv();
    sva::PTransformd X_b_fb = realRobot.X_b1_b2(sensor.parentBody(), fb);
    sva::PTransformd X_s_fb = X_b_fb * X_s_b;
    posW_ = X_s_fb * X_0_s;

    sva::MotionVecd sensorVel(sensor.angularVelocity(), sensor.linearVelocity());
    velW_ = X_s_fb * sensorVel;
  }
  return true;
}

void BodySensorObserver::updateRobot(mc_rbdyn::Robot & realRobot)
{
  if(updateFbFromControl_ || updateFbFromSensor_)
  {
    realRobot.posW(posW_);
    realRobot.velW(velW_);
    realRobot.forwardKinematics();
    realRobot.forwardVelocity();
  }
}

void BodySensorObserver::addToLogger(mc_rtc::Logger & logger)
{
  Observer::addToLogger(logger);
  logger.addLogEntry("observer_" + name() + "_" + fbSensorName_ + "Sensor_posW", [this]() {
    const auto & bs = robot().bodySensor(fbSensorName_);
    return sva::PTransformd(bs.orientation(), bs.position());
  });
  logger.addLogEntry("observer_" + name() + "_" + fbSensorName_ + "Sensor_velW", [this]() {
    const auto & bs = robot().bodySensor(fbSensorName_);
    return sva::MotionVecd(bs.angularVelocity(), bs.linearVelocity());
  });
}
void BodySensorObserver::removeFromLogger(mc_rtc::Logger & logger)
{
  Observer::removeFromLogger(logger);
  logger.removeLogEntry("observer_" + name() + "_" + fbSensorName_ + "Sensor_posW");
  logger.removeLogEntry("observer_" + name() + "_" + fbSensorName_ + "Sensor_velW");
}

void BodySensorObserver::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.addElement({"Observers", name()},
                 mc_rtc::gui::Arrow("Velocity", mc_rtc::gui::ArrowConfig(mc_rtc::gui::Color{1., 0., 0.}),
                                    [this]() {
                                      const auto & bs = robot().bodySensor(fbSensorName_);
                                      return bs.position();
                                    },
                                    [this]() -> Eigen::Vector3d {
                                      const auto & bs = robot().bodySensor(fbSensorName_);
                                      Eigen::Vector3d end = bs.position() + bs.linearVelocity();
                                      return end;
                                    }));
}

} // namespace mc_observers

EXPORT_OBSERVER_MODULE("BodySensor", mc_observers::BodySensorObserver)
