#include "BodySensorObserver.h"

namespace mc_observers
{
BodySensorObserver::BodySensorObserver(const std::string& name, double dt, const mc_rtc::Configuration & config) :
    Observer(name, dt, config)
{
  LOG_INFO("BodySensorObserver config: \n" << config.dump(true));
  updateFb_ = config("UpdateFloatingBase", false);
  fbSensorName_ = config("FloatingBaseSensor", std::string("FloatingBase"));
  updateEncoderPos_ = config("UpdateEncoderPosition", false);
  updateEncoderVel_ = config("UpdateEncoderVelocity", false);
}

void BodySensorObserver::reset(const mc_rbdyn::Robot & controlRobot, const mc_rbdyn::Robot & realRobot)
{
}

bool BodySensorObserver::run(const mc_rbdyn::Robot & controlRobot, const mc_rbdyn::Robot & realRobot)
{
  return true;
}

void BodySensorObserver::updateRobot(mc_rbdyn::Robot & realRobot)
{
  if(updateFb_)
  {
    const auto & bs = realRobot.bodySensor(fbSensorName_);
    posW_ = {bs.orientation(), bs.position()};
    velW_ = {bs.angularVelocity(), bs.linearVelocity()};
    realRobot.posW(posW_);
    realRobot.velW(velW_);
  }

  if(updateEncoderPos_)
  {
    const auto& q = realRobot.encoderValues();
    if(q.size() == realRobot.refJointOrder().size())
    {
      // Set all joint values and velocities from encoders
      unsigned i = 0;
      for(const auto & ref_joint : realRobot.refJointOrder())
      {
        if(realRobot.hasJoint(ref_joint))
        {
          const auto joint_index = static_cast<size_t>(realRobot.mb().jointIndexByName(ref_joint));
          if(!q.empty())
          {
            realRobot.mbc().q[joint_index][0] = q[i];
          }
        }
        i++;
      }
    }
  }

  if(updateEncoderVel_)
  {
    const auto& alpha = realRobot.encoderVelocities();
    if(alpha.size() == realRobot.refJointOrder().size())
    {
      // Set all joint values and velocities from encoders
      unsigned i = 0;
      for(const auto & ref_joint : realRobot.refJointOrder())
      {
        if(realRobot.hasJoint(ref_joint))
        {
          const auto joint_index = static_cast<size_t>(realRobot.mb().jointIndexByName(ref_joint));
          if(!alpha.empty())
          {
            realRobot.mbc().q[joint_index][0] = alpha[i];
          }
        }
        i++;
      }
    }

  }
}

void BodySensorObserver::addToLogger(mc_rtc::Logger &logger)
{
  Observer::addToLogger(logger);
  logger.addLogEntry("observer_"+name()+"_posW",
                     [this]()
                     {
                       return posW_;
                     });
  logger.addLogEntry("observer_"+name()+"_velW",
                     [this]()
                     {
                       return velW_;
                     });
}
void BodySensorObserver::removeFromLogger(mc_rtc::Logger &logger)
{
  Observer::removeFromLogger(logger);
  logger.removeLogEntry("observer_"+name()+"_posW");
  logger.removeLogEntry("observer_"+name()+"_velW");
}

void BodySensorObserver::addToGUI(mc_rtc::gui::StateBuilder &gui)
{
  if(updateFb_)
  {
    gui.addElement({"Observers", name()},
                   mc_rtc::gui::Arrow("Velocity",
                                      mc_rtc::gui::ArrowConfig(mc_rtc::gui::Color{1.,0.,0.}),
                                      [this]()
                                      {
                                      return posW_.translation();
                                      },
                                      [this]() -> Eigen::Vector3d
                                      {
                                      const auto & p = posW_.translation();
                                      LOG_INFO("p: " << p.transpose());
                                      Eigen::Vector3d end = p + velW_.linear();
                                      return end;
                                      })
                  );
  }
}

} // namespace mc_observers

EXPORT_OBSERVER_MODULE("BodySensor", mc_observers::BodySensorObserver)
