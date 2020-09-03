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
  updateRobot_ = config("updateRobot", static_cast<std::string>(robot_));
  if(!ctl.robots().hasRobot(robot_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Observer {} requires robot \"{}\" but this robot does not exit",
                                                     name(), robot_);
  }
  if(!ctl.robots().hasRobot(updateRobot_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "Observer {} requires robot \"{}\" (updateRobot) but this robot does not exit", name(), updateRobot_);
  }
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

  config("computeFK", computeFK_);
  config("computeFV", computeFV_);

  if(config.has("log"))
  {
    auto lConfig = config("log");
    lConfig("position", logPosition_);
    lConfig("velocity", logVelocity_);
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

void EncoderObserver::update(mc_control::MCController & ctl)
{
  auto & realRobots = ctl.realRobots();
  const auto & robot = ctl.robots().robot(robot_);
  auto & realRobot = realRobots.robot(updateRobot_);
  const auto & q = robot.encoderValues();

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
  if(computeFK_ && posUpdate_ != PosUpdate::None)
  {
    realRobot.forwardKinematics();
  }
  if(computeFV_ && velUpdate_ != VelUpdate::None)
  {
    realRobot.forwardVelocity();
  }
}

void EncoderObserver::addToLogger(const mc_control::MCController & ctl,
                                  mc_rtc::Logger & logger,
                                  const std::string & category)
{
  if(logPosition_)
  {
    if(posUpdate_ == PosUpdate::EncoderValues)
    {
      logger.addLogEntry(category + "_encoderValues", [this, &ctl]() { return ctl.robot(robot_).encoderValues(); });
    }
    else if(velUpdate_ == VelUpdate::Control)
    {
      std::vector<double> qOut(ctl.robot(robot_).refJointOrder().size(), 0);
      logger.addLogEntry(category + "_controlValues", [this, &ctl, qOut]() mutable -> const std::vector<double> & {
        for(size_t i = 0; i < qOut.size(); ++i)
        {
          auto jIdx = ctl.robot(robot_).jointIndexInMBC(i);
          if(jIdx != -1)
          {
            qOut[i] = ctl.robot(robot_).mbc().alpha[static_cast<size_t>(jIdx)][0];
          }
        }
        return qOut;
      });
    }
  }

  if(logVelocity_)
  {
    if(velUpdate_ == VelUpdate::EncoderFiniteDifferences)
    {
      logger.addLogEntry(category + "_encoderFiniteDifferences", [this]() { return encodersVelocity_; });
    }
    else if(velUpdate_ == VelUpdate::EncoderVelocities)
    {
      logger.addLogEntry(category + "_encoderVelocities",
                         [this, &ctl]() { return ctl.robot(robot_).encoderVelocities(); });
    }
    else if(velUpdate_ == VelUpdate::Control)
    {
      std::vector<double> alpha(ctl.robot(robot_).refJointOrder().size(), 0);
      logger.addLogEntry(category + "_controlVelocities", [this, &ctl, alpha]() mutable -> const std::vector<double> & {
        for(size_t i = 0; i < alpha.size(); ++i)
        {
          auto jIdx = ctl.robot(robot_).jointIndexInMBC(i);
          if(jIdx != -1)
          {
            alpha[i] = ctl.robot(robot_).mbc().alpha[static_cast<size_t>(jIdx)][0];
          }
        }
        return alpha;
      });
    }
  }
}
void EncoderObserver::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category + "_" + name() + "encoderValues");
  logger.removeLogEntry(category + "_" + name() + "controlValues");
  logger.removeLogEntry(category + "_" + name() + "encoderFiniteDifferences");
  logger.removeLogEntry(category + "_" + name() + "encoderVelocities");
  logger.removeLogEntry(category + "_" + name() + "controlVelocities");
}

} // namespace mc_observers

EXPORT_OBSERVER_MODULE("Encoder", mc_observers::EncoderObserver)
