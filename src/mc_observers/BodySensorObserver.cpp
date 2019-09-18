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
  updateEncoderPosFromSensor_ = config("UpdateEncoderPositionFromSensor", true);
  updateEncoderVelFromSensor_ = config("UpdateEncoderVelocityFromSensor", true);
  updateEncoderPosFromControl_ = config("UpdateEncoderPositionFromControl", false);
  updateEncoderVelFromControl_ = config("UpdateEncoderVelocityFromControl", false);
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
    const auto & bs = robot().bodySensor(fbSensorName_);
    posW_ = {bs.orientation(), bs.position()};
    velW_ = {bs.angularVelocity(), bs.linearVelocity()};
  }
  return true;
}

void BodySensorObserver::updateRobot(mc_rbdyn::Robot & realRobot)
{
  if(updateFbFromControl_ || updateFbFromSensor_)
  {
    realRobot.posW(posW_);
    realRobot.velW(velW_);
  }

  const auto & q = robot().encoderValues();
  const auto & alpha = robot().encoderVelocities();
  if(q.size() == robot().refJointOrder().size())
  {
    // Set all joint values and velocities from encoders
    unsigned i = 0;
    for(const auto & ref_joint : realRobot.refJointOrder())
    {
      if(robot().hasJoint(ref_joint))
      {
        const auto joint_index = static_cast<size_t>(robot().mb().jointIndexByName(ref_joint));
        if(updateEncoderPosFromControl_)
        {
          realRobot.mbc().q[joint_index][0] = robot().mbc().q[joint_index][0];
        }
        if(updateEncoderPosFromSensor_)
        {
          if(!q.empty())
          {
            realRobot.mbc().q[joint_index][0] = q[i];
          }
        }

        if(updateEncoderPosFromControl_)
        {
          realRobot.mbc().alpha[joint_index][0] = robot().mbc().alpha[joint_index][0];
        }
        if(updateEncoderVelFromSensor_)
        {
          if(!alpha.empty())
          {
            realRobot.mbc().alpha[joint_index][0] = alpha[i];
          }
        }
      }
      i++;
    }
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
