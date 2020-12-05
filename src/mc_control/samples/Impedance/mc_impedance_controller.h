/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/api.h>
#include <mc_control/mc_controller.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/ImpedanceTask.h>

namespace mc_control
{

struct MC_CONTROL_DLLAPI MCImpedanceController : public MCController
{
  MCImpedanceController(std::shared_ptr<mc_rbdyn::RobotModule> robot, double dt);
  void reset(const ControllerResetData & reset_data) override;
  bool run() override;

  void supported_robots(std::vector<std::string> & out) const override
  {
    // Hand surface name is specific to the robot
    out = {"jvrc1"};
  }

protected:
  std::shared_ptr<mc_tasks::EndEffectorTask> rootLinkTask;
  std::shared_ptr<mc_tasks::force::ImpedanceTask> impedanceTask;

  double angle = 0.0;
  double radius = 0.2;
  Eigen::Vector3d center = Eigen::Vector3d(0.3, 0.5, 1.0);
};

} // namespace mc_control

SIMPLE_CONTROLLER_CONSTRUCTOR("Impedance", mc_control::MCImpedanceController)
