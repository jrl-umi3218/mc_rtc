/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/RobotModule.h>

#include "api.h"

namespace mc_robots
{

struct MC_ROBOTS_DLLAPI JVRC1RobotModule : public mc_rbdyn::RobotModule
{
public:
  JVRC1RobotModule();
};

} // namespace mc_robots
