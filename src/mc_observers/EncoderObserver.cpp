/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "EncoderObserver.h"

#include <mc_rbdyn/rpy_utils.h>

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

  LOG_SUCCESS("EncoderObserver created")
}

void EncoderObserver::reset(const mc_rbdyn::Robot & realRobot)
{
  const auto & enc = robot().encoderValues();
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

bool EncoderObserver::run(const mc_rbdyn::Robot & realRobot)
{
  const auto & enc = robot().encoderValues();
  for(unsigned i = 0; i < enc.size(); ++i)
  {
    encodersVelocity_[i] = (enc[i] - prevEncoders_[i]) / dt();
    prevEncoders_[i] = enc[i];
  }
  return true;
}

void EncoderObserver::updateRobot(mc_rbdyn::Robot & realRobot)
{
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
        // Update position
        if(posUpdate_ == Update::Control)
        {
          realRobot.mbc().q[joint_index][0] = robot().mbc().q[joint_index][0];
        }
        else if(posUpdate_ == Update::Estimator)
        {
          realRobot.mbc().q[joint_index][0] = q[i];
        }

        // Update velocity
        if(velUpdate_ == Update::Control)
        {
          realRobot.mbc().alpha[joint_index][0] = robot().mbc().alpha[joint_index][0];
        }
        else if(velUpdate_ == Update::Estimator)
        {
          realRobot.mbc().alpha[joint_index][0] = encodersVelocity_[i];
        }
      }
      i++;
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

void EncoderObserver::addToLogger(mc_rtc::Logger & logger)
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
void EncoderObserver::addToGUI(mc_rtc::gui::StateBuilder & gui) {}
void EncoderObserver::removeFromGUI(mc_rtc::gui::StateBuilder & gui) {}

} // namespace mc_observers

EXPORT_OBSERVER_MODULE("Encoder", mc_observers::EncoderObserver)
