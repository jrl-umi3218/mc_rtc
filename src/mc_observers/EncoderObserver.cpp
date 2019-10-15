/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/mc_controller.h>
#include <mc_observers/EncoderObserver.h>

namespace mc_observers
{
EncoderObserver::EncoderObserver(const std::string & name, double dt, const mc_rtc::Configuration & config)
: Observer(name, dt, config)
{
  const std::string & position = config("UpdatePosition", std::string("estimator"));
  if(position == "control")
  {
    posUpdate_ = Update::Control;
  }
  else if(position == "estimator")
  {
    posUpdate_ = Update::Estimator;
  }
  else if(position == "none")
  {
    posUpdate_ = Update::None;
  }
  else
  {
    LOG_WARNING("[EncoderObserver] Invalid configuration value for UpdatePosition (valid values are [control, sensor, "
                "none]), using default behaviour (updating joint velocity from estimator) ");
    posUpdate_ = Update::Estimator;
  }

  const std::string & velocity = config("UpdateVelocity", std::string("estimator"));
  if(velocity == "control")
  {
    velUpdate_ = Update::Control;
  }
  else if(velocity == "estimator")
  {
    velUpdate_ = Update::Estimator;
  }
  else if(velocity == "none")
  {
    velUpdate_ = Update::None;
  }
  else
  {
    LOG_WARNING("[EncoderObserver] Invalid configuration value for UpdateVelocity (valid values are [control, sensor, "
                "none]), using default behaviour (updating joint velocity from estimator) ");
    velUpdate_ = Update::Estimator;
  }

  logEstimation_ = config("Log", false);

  desc_ = name_ + " (position=" + position + ",velocity=" + velocity + ")";
}

void EncoderObserver::reset(const mc_control::MCController & ctl)
{
  const auto & enc = ctl.robot().encoderValues();
  if(enc.empty() && (posUpdate_ == Update::Estimator || velUpdate_ == Update::Estimator))
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "[EncoderObserver] requires the robot to have encoder measurements")
  }

  if(!enc.empty())
  {
    prevEncoders_ = enc;
    encodersVelocity_.resize(enc.size());
    for(unsigned i = 0; i < enc.size(); ++i)
    {
      encodersVelocity_[i] = 0;
    }
  }
}

bool EncoderObserver::run(const mc_control::MCController & ctl)
{
  const auto & enc = ctl.robot().encoderValues();
  for(unsigned i = 0; i < enc.size(); ++i)
  {
    encodersVelocity_[i] = (enc[i] - prevEncoders_[i]) / dt();
    prevEncoders_[i] = enc[i];
  }
  return true;
}

void EncoderObserver::updateRobots(const mc_control::MCController & ctl, mc_rbdyn::Robots & realRobots)
{
  const auto & robot = ctl.robot();
  auto & realRobot = realRobots.robot();
  const auto & q = robot.encoderValues();

  if(q.size() == robot.refJointOrder().size())
  {
    // Set all joint values and velocities from encoders
    size_t nJoints = realRobot.refJointOrder().size();
    for(size_t i = 0; i < nJoints; ++i)
    {
      const auto joint_index = robot.jointIndexInMBC(i);
      if(joint_index != -1 && robot.mb().joint(joint_index).dof() == 1)
      {
        size_t jidx = static_cast<size_t>(joint_index);
        // Update position
        if(posUpdate_ == Update::Control)
        {
          realRobot.mbc().q[jidx][0] = robot.mbc().q[jidx][0];
        }
        else if(posUpdate_ == Update::Estimator)
        {
          realRobot.mbc().q[jidx][0] = q[i];
        }

        // Update velocity
        if(velUpdate_ == Update::Control)
        {
          realRobot.mbc().alpha[jidx][0] = robot.mbc().alpha[jidx][0];
        }
        else if(velUpdate_ == Update::Estimator)
        {
          realRobot.mbc().alpha[jidx][0] = encodersVelocity_[i];
        }
      }
    }
  }
  if(posUpdate_ != Update::None)
  {
    realRobot.forwardKinematics();
  }
  if(velUpdate_ != Update::None)
  {
    realRobot.forwardVelocity();
  }
}

void EncoderObserver::addToLogger(const mc_control::MCController & /* ctl */, mc_rtc::Logger & logger)
{
  if(logEstimation_)
  {
    logger.addLogEntry("observer_" + name() + "_alpha", [this]() { return encodersVelocity_; });
  }
}
void EncoderObserver::removeFromLogger(mc_rtc::Logger & logger)
{
  if(logEstimation_)
  {
    logger.removeLogEntry("observer_" + name() + "_alpha");
  }
}

} // namespace mc_observers

EXPORT_OBSERVER_MODULE("Encoder", mc_observers::EncoderObserver)
