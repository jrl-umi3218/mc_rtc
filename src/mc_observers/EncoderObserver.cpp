/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/MCController.h>
#include <mc_observers/EncoderObserver.h>
#include <mc_observers/ObserverMacros.h>

namespace mc_observers
{

void EncoderObserver::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  robot_ = config("robot", ctl.robot().name());
  const std::string & position = config("position", std::string("encoderValues"));
  if(position == "control")
  {
    posUpdate_ = PosUpdate::Control;
  }
  else if(position == "encoderValues")
  {
    posUpdate_ = PosUpdate::EncoderValues;
  }
  else if(position == "none")
  {
    posUpdate_ = PosUpdate::None;
  }
  else
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[EncoderObserver::{}] Invalid configuration value \"{}\" for field \"position\" (valid values are [control, "
        "encoderValues, none])",
        name_, position);
  }

  const std::string & velocity = config("velocity", std::string("encoderFiniteDifferences"));
  if(velocity == "control")
  {
    velUpdate_ = VelUpdate::Control;
  }
  else if(velocity == "encoderFiniteDifferences")
  {
    velUpdate_ = VelUpdate::EncoderFiniteDifferences;
  }
  else if(velocity == "encoderVelocities")
  {
    velUpdate_ = VelUpdate::EncoderVelocities;
  }
  else if(velocity == "none")
  {
    velUpdate_ = VelUpdate::None;
  }
  else
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[EncoderObserver::{}] Invalid configuration value \"{}\" for field \"velocity\" (valid values are [control, "
        "encoderFiniteDifferences, encoderVelocities, none])",
        name_, velocity);
    ;
  }

  desc_ = name_ + " (position=" + position + ",velocity=" + velocity + ")";
}

void EncoderObserver::reset(const mc_control::MCController & ctl)
{
  auto & robot = ctl.robots().robot(robot_);
  const auto & enc = robot.encoderValues();
  if(enc.empty()
     && (posUpdate_ == PosUpdate::EncoderValues || velUpdate_ == VelUpdate::EncoderFiniteDifferences
         || velUpdate_ == VelUpdate::EncoderVelocities))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[EncoderObserver] requires robot {} to have encoder measurements",
                                                     robot_);
  }
  if(velUpdate_ == VelUpdate::EncoderVelocities && robot.encoderVelocities().empty())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[EncoderObserver] requires robot {} to have encoder velocity measurements", robot_);
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
  auto & robot = ctl.robots().robot(robot_);
  if(velUpdate_ == VelUpdate::EncoderFiniteDifferences)
  {
    const auto & enc = robot.encoderValues();
    for(unsigned i = 0; i < enc.size(); ++i)
    {
      encodersVelocity_[i] = (enc[i] - prevEncoders_[i]) / ctl.timeStep;
      prevEncoders_[i] = enc[i];
    }
  }
  return true;
}

void EncoderObserver::updateRobots(mc_control::MCController & ctl)
{
  auto & realRobots = ctl.realRobots();
  const auto & robot = ctl.robots().robot(robot_);
  auto & realRobot = realRobots.robot(robot_);
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
        if(posUpdate_ == PosUpdate::Control)
        {
          realRobot.mbc().q[jidx][0] = robot.mbc().q[jidx][0];
        }
        else if(posUpdate_ == PosUpdate::EncoderValues)
        {
          realRobot.mbc().q[jidx][0] = q[i];
        }

        // Update velocity
        if(velUpdate_ == VelUpdate::Control)
        {
          realRobot.mbc().alpha[jidx][0] = robot.mbc().alpha[jidx][0];
        }
        else if(velUpdate_ == VelUpdate::EncoderFiniteDifferences)
        {
          realRobot.mbc().alpha[jidx][0] = encodersVelocity_[i];
        }
        else if(velUpdate_ == VelUpdate::EncoderVelocities)
        {
          realRobot.mbc().alpha[jidx][0] = robot.encoderVelocities()[i];
        }
      }
    }
  }
  if(posUpdate_ != PosUpdate::None)
  {
    realRobot.forwardKinematics();
  }
  if(velUpdate_ != VelUpdate::None)
  {
    realRobot.forwardVelocity();
  }
}

void EncoderObserver::addToLogger(mc_control::MCController & ctl, std::string category)
{
  if(velUpdate_ == VelUpdate::EncoderFiniteDifferences)
  {
    ctl.logger().addLogEntry(category + "_" + name() + "_encoderFiniteDifferences",
                             [this]() { return encodersVelocity_; });
  }
  if(velUpdate_ == VelUpdate::EncoderVelocities)
  {
    ctl.logger().addLogEntry(category + "_" + name() + "_encoderVelocities",
                             [this, &ctl]() { return ctl.robots().robot(robot_).encoderVelocities(); });
  }
}
void EncoderObserver::removeFromLogger(mc_control::MCController & ctl, std::string category)
{
  if(velUpdate_ == VelUpdate::EncoderFiniteDifferences)
  {
    ctl.logger().removeLogEntry(category + "_" + name() + "_encoderFiniteDifferences");
  }
  if(velUpdate_ == VelUpdate::EncoderVelocities)
  {
    ctl.logger().removeLogEntry(category + "_" + name() + "_encoderVelocities");
  }
}

} // namespace mc_observers

EXPORT_OBSERVER_MODULE("Encoder", mc_observers::EncoderObserver)
