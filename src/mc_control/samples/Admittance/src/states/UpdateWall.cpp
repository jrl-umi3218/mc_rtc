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
  if(!config_.has("surface"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Missing required configuration for \"surface\"", name());
  }
  if(!config_.has("moveRobot"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Missing required configuration for \"moveRobot\"", name());
  }
  const auto rName = config_("robot", ctl.robot().name());
  const auto sName = config_("surface");
  const auto moveRobotName = config_("moveRobot");
  if(!ctl.robots().hasRobot(rName))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] No robot named {}", name(), rName);
  }
  if(!ctl.robot(rName).hasSurface(sName))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] No surface named {} in robot {}", name(), sName, rName);
  }
  if(!ctl.robots().hasRobot(moveRobotName))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] No robot named {}", name(), moveRobotName);
  }

  const auto surfacePose = ctl.robot(rName).surfacePose(sName);
  auto posW = ctl.robot(moveRobotName).posW();
  posW.translation().x() = surfacePose.translation().x();
  ctl.robot(moveRobotName).posW(posW);
}

bool UpdateWall::run(mc_control::fsm::Controller &)
{
  output("OK");
  return true;
}

void UpdateWall::teardown(mc_control::fsm::Controller &) {}

EXPORT_SINGLE_STATE("UpdateWall", UpdateWall)
