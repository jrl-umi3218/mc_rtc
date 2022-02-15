/*
 * Copyright 2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "UpdateWall.h"
#include <mc_control/fsm/Controller.h>
#include <mc_rtc/ConfigurationHelpers.h>

void UpdateWall::configure(const mc_rtc::Configuration & config)
{
  config_.load(config);
}

void UpdateWall::start(mc_control::fsm::Controller & ctl)
{
  if(!config_.has("body"))
  {
    mc_rtc::log::error_and_throw("[{}] Missing required configuration for \"body\"", name());
  }
  if(!config_.has("moveRobot"))
  {
    mc_rtc::log::error_and_throw("[{}] Missing required configuration for \"moveRobot\"", name());
  }
  const auto rName = config_("robot", ctl.robot().name());
  const auto bName = config_("body");
  const auto moveRobotName = config_("moveRobot");
  if(!ctl.realRobots().hasRobot(rName))
  {
    mc_rtc::log::error_and_throw("[{}] No robot named {}", name(), rName);
  }
  if(!ctl.realRobots().robot(rName).hasBody(bName))
  {
    mc_rtc::log::error_and_throw("[{}] No body named {} in robot {}", name(), bName, rName);
  }
  if(!ctl.robots().hasRobot(moveRobotName))
  {
    mc_rtc::log::error_and_throw("[{}] No robot named {}", name(), moveRobotName);
  }

  const auto & bodyPose = ctl.realRobots().robot(rName).bodyPosW(bName);
  auto posW = ctl.robot(moveRobotName).posW();
  posW.translation().x() = bodyPose.translation().x();
  ctl.robot(moveRobotName).posW(posW);
}

bool UpdateWall::run(mc_control::fsm::Controller &)
{
  output("OK");
  return true;
}

void UpdateWall::teardown(mc_control::fsm::Controller &) {}

EXPORT_SINGLE_STATE("UpdateWall", UpdateWall)
