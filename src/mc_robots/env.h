/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/RobotModule.h>
#include <mc_rtc/config.h>

#include "api.h"

namespace mc_robots
{

struct MC_ROBOTS_DLLAPI EnvRobotModule : public mc_rbdyn::RobotModule
{
public:
  EnvRobotModule(const std::string & env_path, const std::string & env_name, bool fixed);
};

} // namespace mc_robots
