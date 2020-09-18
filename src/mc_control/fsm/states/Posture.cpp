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
    has_postureTask_ = true;
    postureTaskConfig_.load(config("postureTask"));
  }
  if(config.has("robot"))
  {
    has_robot_ = true;
    robot_ = static_cast<std::string>(config("robot"));
  }
}

void PostureState::start(Controller & ctl)
{
  if(!has_postureTask_)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] requires a \"postureTask\" element", name());
  }
  if(!has_robot_)
  {
    robot_ = ctl.robot().name();
    if(!ctl.robots().hasRobot(robot_))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[{}] No robot named \"{}\"", name(), robot_);
    }
  }
  postureTaskConfig_.add("robot", robot_);
  postureTaskConfig_.add("type", "posture");
  postureTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::PostureTask>(ctl.solver(), postureTaskConfig_);
  ctl.solver().removeTask(ctl.getPostureTask(robot_));
  ctl.solver().addTask(postureTask_);

  crit_.configure(*postureTask_, ctl.solver().dt(), postureTaskConfig_("completion"));
}

bool PostureState::run(Controller &)
{
  if(crit_.completed(*postureTask_))
  {
    output("OK");
    return true;
  }
  return false;
}

void PostureState::teardown(Controller & ctl)
{
  ctl.solver().addTask(ctl.getPostureTask(robot_));
  ctl.solver().removeTask(postureTask_);
}

} // namespace fsm

} // namespace mc_control

EXPORT_SINGLE_STATE("Posture", mc_control::fsm::PostureState)
