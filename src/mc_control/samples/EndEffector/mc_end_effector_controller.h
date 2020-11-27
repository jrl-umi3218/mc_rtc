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
  MCEndEffectorController(std::shared_ptr<mc_rbdyn::RobotModule> robot,
                          double dt,
                          const mc_rtc::Configuration & config);

  void reset(const ControllerResetData & reset_data) override;

private:
  std::shared_ptr<mc_tasks::EndEffectorTask> efTask_;
  std::shared_ptr<mc_tasks::CoMTask> comTask_;
  std::vector<mc_tasks::MetaTaskPtr> tasks_;
};

} // namespace mc_control

CONTROLLER_CONSTRUCTOR("EndEffector", mc_control::MCEndEffectorController)
