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
  std::shared_ptr<mc_tasks::force::ImpedanceTask> impedanceTask_;
  double angle_ = 0.0;
  double radius_ = 0.2;
  double speed_ = 1.0;
  Eigen::Vector3d center_ = Eigen::Vector3d(0.3, 0.5, 1.0);
  Eigen::Matrix3d orientation_ = Eigen::Matrix3d::Identity();
};

} // namespace mc_control

SIMPLE_CONTROLLER_CONSTRUCTOR("Impedance", mc_control::MCImpedanceController)
