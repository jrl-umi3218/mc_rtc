#pragma once

#include <mc_control/mc_controller.h>
#include <mc_tasks/SurfaceTransformTask.h>
#include <mc_tasks/TransformTask.h>

#include "api.h"

enum Phase
{
  APPROACH = 0,
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
  Phase phase_ = APPROACH;
  std::shared_ptr<mc_tasks::SurfaceTransformTask> handTask_;
  std::shared_ptr<mc_tasks::TransformTask> dingoBaseTask_;
  std::shared_ptr<mc_solver::KinematicsConstraint> doorKinematics_;
  std::shared_ptr<mc_tasks::PostureTask> doorPostureTask_;
};
