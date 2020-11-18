/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/api.h>
#include <mc_control/mc_controller.h>

#include <mc_tasks/CoMTask.h>
#include <mc_tasks/EndEffectorTask.h>

namespace mc_control
{

struct MC_CONTROL_DLLAPI MCEndEffectorController : public MCController
{
public:
  MCEndEffectorController(std::shared_ptr<mc_rbdyn::RobotModule> robot, double dt);

  void reset(const ControllerResetData & reset_data) override;

public:
  std::shared_ptr<mc_tasks::EndEffectorTask> efTask;
  std::shared_ptr<mc_tasks::CoMTask> comTask;
};

} // namespace mc_control

SIMPLE_CONTROLLER_CONSTRUCTOR("EndEffector", mc_control::MCEndEffectorController)
