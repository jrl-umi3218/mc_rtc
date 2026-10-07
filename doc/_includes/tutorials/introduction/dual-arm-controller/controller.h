#pragma once

#include <mc_control/mc_controller.h>
#include <mc_solver/CollisionsConstraint.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/PostureTask.h>

#include "api.h"

enum ControllerPhase
{
  IDLE = 0,
  STARTED,
  MOVE
};
enum ControllerState
{
  GO = 0,
  RETURN
};

struct DualArmController_DLLAPI DualArmController : public mc_control::MCController
{
public:
  DualArmController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

private:
  std::shared_ptr<mc_tasks::EndEffectorTask> urEndEffectorTask_;

  std::shared_ptr<mc_tasks::PostureTask> kinovaPostureTask_;
  std::unique_ptr<mc_solver::KinematicsConstraint> kinovaKinematics_;

  const double iDist = 0.1;
  const double sDist = 0.05;
  const double damping = 0.0;

  ControllerPhase phase_ = IDLE;

  ControllerState urState_ = RETURN;
  ControllerState kinovaState_ = RETURN;

  void runUr();
  void runKinova();
};
