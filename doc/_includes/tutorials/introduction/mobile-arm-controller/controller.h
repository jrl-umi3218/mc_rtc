#pragma once

#include <mc_control/mc_controller.h>
#include <mc_solver/KinematicsConstraint.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/PostureTask.h>
#include <mc_tasks/SurfaceTransformTask.h>

#include "api.h"

enum Phase
{
  APPROACH,
  HANDLE,
  OPEN,
  DONE
};

struct MobileArmController_DLLAPI MobileArmController : public mc_control::MCController
{
public:
  MobileArmController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

private:
  int count = 0;
  Phase phase = APPROACH;
  std::shared_ptr<mc_tasks::SurfaceTransformTask> handTask_;
  std::shared_ptr<mc_tasks::EndEffectorTask> dingoEndEffectorTask_;
  std::shared_ptr<mc_solver::KinematicsConstraint> dingoDynamics_;
  std::shared_ptr<mc_solver::KinematicsConstraint> doorKinematics_;
  std::shared_ptr<mc_tasks::PostureTask> doorPostureTask_;
};
