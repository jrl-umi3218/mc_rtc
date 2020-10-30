/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/api.h>
#include <mc_control/mc_controller.h>

namespace mc_control
{

struct MC_CONTROL_DLLAPI MCPostureController : public MCController
{
public:
  /* Common stuff */
  MCPostureController(std::shared_ptr<mc_rbdyn::RobotModule> robot, double dt);

  bool run() override;
};

} // namespace mc_control
