/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/api.h>
#include <mc_control/mc_controller.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/PostureTask.h>

namespace mc_control
{

struct MC_CONTROL_DLLAPI MCCoMController : public MCController
{
  MCCoMController(std::shared_ptr<mc_rbdyn::RobotModule> robot, double dt);
  void reset(const ControllerResetData & reset_data) override;
  bool run() override;

protected:
  void updateAnchorFrame();

  std::shared_ptr<mc_tasks::CoMTask> comTask;
  std::string leftFootSurface, rightFootSurface;
};

} // namespace mc_control

SIMPLE_CONTROLLER_CONSTRUCTOR("CoM", mc_control::MCCoMController)
