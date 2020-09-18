/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/states/Posture.h>
#include <mc_rbdyn/configuration_io.h>
#include <mc_tasks/MetaTaskLoader.h>

namespace mc_control
{

namespace fsm
{

void PostureState::configure(const mc_rtc::Configuration & config)
{
  if(config.has("postureTask"))
  {
    config_.load(config("postureTask"));
  }
  if(config.has("robot"))
  {
    robot_ = static_cast<std::string>(config("robot"));
  }
  if(config.has("completion"))
  {
    hasCompletion_ = true;
    completion_ = config("completion");
  }
}

void PostureState::start(Controller & ctl)
{
  if(robot_.empty())
  {
    robot_ = ctl.robot().name();
  }
  else if(!ctl.hasRobot(robot_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Requested robot {} but this robot is not available", name(),
                                                     robot_);
  }

  auto postureTask = ctl.getPostureTask(robot_);
  postureTask->load(ctl.solver(), config_);
  crit_.configure(*postureTask, ctl.solver().dt(), completion_);
}

bool PostureState::run(Controller & ctl)
{
  auto postureTask = ctl.getPostureTask(robot_);
  if(crit_.completed(*postureTask))
  {
    output("OK");
    return true;
  }
  return false;
}

void PostureState::teardown(Controller &) {}

} // namespace fsm

} // namespace mc_control

EXPORT_SINGLE_STATE("Posture", mc_control::fsm::PostureState)
