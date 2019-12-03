/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/api.h>
#include <mc_control/mc_controller.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/EndEffectorTask.h>

namespace mc_control
{

struct MC_CONTROL_DLLAPI MCBody6dController : public MCController
{
public:
  MCBody6dController(std::shared_ptr<mc_rbdyn::RobotModule> robot, double dt);

  virtual void reset(const ControllerResetData & reset_data) override;

public:
  std::shared_ptr<mc_tasks::EndEffectorTask> efTask;
  std::shared_ptr<mc_tasks::CoMTask> comTask;
};

} // namespace mc_control

SIMPLE_CONTROLLER_CONSTRUCTOR("Body6d", mc_control::MCBody6dController)
