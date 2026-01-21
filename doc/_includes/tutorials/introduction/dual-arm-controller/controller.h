#pragma once

#include <mc_control/mc_controller.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/PostureTask.h>
#include <mc_solver/CollisionsConstraint.h>

#include "api.h"

enum ControllerPhase { IDLE = 0, STARTED, MOVE };
enum ControllerState { GO= 0, RETURN };

struct DualArmController_DLLAPI DualArmController
    : public mc_control::MCController {
public:
  DualArmController(mc_rbdyn::RobotModulePtr rm, double dt,
                    const mc_rtc::Configuration &config);

  bool run() override;

  void reset(const mc_control::ControllerResetData &reset_data) override;

private:
  void runUr();
  void runKinova();
  ControllerPhase phase_ = IDLE;
  ControllerState urState_ = RETURN;
  ControllerState kinovaState_ = RETURN;
  std::shared_ptr<mc_tasks::PostureTask> kinovaPostureTask_;
  std::shared_ptr<mc_tasks::EndEffectorTask> urEndEffectorTask_;
  std::shared_ptr<mc_solver::CollisionsConstraint> collisionConstraint_;
  std::vector<std::string> urJoints_ = {
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint"};
  double iDist = 0.1;
  double sDist = 0.05;
};