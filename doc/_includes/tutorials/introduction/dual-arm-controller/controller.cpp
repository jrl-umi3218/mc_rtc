#include "DualArmController.h"

#include <mc_rbdyn/RobotLoader.h>

DualArmController::DualArmController(mc_rbdyn::RobotModulePtr rm, double dt,
                                     const mc_rtc::Configuration &config)
    : mc_control::MCController({rm, mc_rbdyn::RobotLoader::get_robot_module(
                                        "env", "/usr/local/share/mc_kinova",
                                        std::string("kinova_default"))},
                               dt) {
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(kinematicsConstraint);
  solver().addConstraintSet(selfCollisionConstraint);
  addCollisions(
    "ur5e", "kinova_default", {{"*", "*", iDist, sDist, 0}}
  );
  postureTask->stiffness(1);
  postureTask->weight(1);
  solver().addTask(postureTask.get());
  solver().setContacts({{}});
}

bool DualArmController::run() {
  if (phase_ == IDLE) {
    postureTask->target(
          {{"elbow_joint", {-M_PI / 2}}, {"wrist_2_joint", {M_PI / 2}}});
    phase_ = STARTED;
  }
  else if (phase_ == STARTED && postureTask->eval().norm() < 0.01 &&
      postureTask->speed().norm() < 0.01) {
    phase_ = MOVE;
    urEndEffectorTask_->selectActiveJoints(solver(), urJoints_);
  } else if (phase_ == STARTED) {
    urEndEffectorTask_->reset();
  }
  else if (phase_ == MOVE) {
    runUr();
    runKinova();
  }
  return mc_control::MCController::run();
}

void DualArmController::runUr() {
  if (urState_ == GO && urEndEffectorTask_->eval().norm() < 0.05 &&
      urEndEffectorTask_->speed().norm() < 0.01) {
    urState_ = RETURN;
    urEndEffectorTask_->add_ef_pose({Eigen::Vector3d(0.0, -0.5, 0.0)});
  } else if (urState_ == RETURN && urEndEffectorTask_->eval().norm() < 0.01 &&
             urEndEffectorTask_->speed().norm() < 0.05) {
    urState_ = GO;
    urEndEffectorTask_->add_ef_pose({Eigen::Vector3d(0.0, 0.5, 0.0)});
  }
}

void DualArmController::runKinova() {
  if (kinovaState_ == GO && kinovaPostureTask_->eval().norm() < 0.01 &&
      kinovaPostureTask_->speed().norm() < 0.01) {
    kinovaState_ = RETURN;
    kinovaPostureTask_->target({{"joint_2", {0.0}}});
  } else if (kinovaState_ == RETURN &&
             kinovaPostureTask_->eval().norm() < 0.01 &&
             kinovaPostureTask_->speed().norm() < 0.01) {
    kinovaState_ = GO;
    kinovaPostureTask_->target({{"joint_2", {-M_PI / 4}}});
  }
}

void DualArmController::reset(
    const mc_control::ControllerResetData &reset_data) {
  mc_control::MCController::reset(reset_data);
  postureTask->reset();
  std::string urBody = "wrist_3_link";
  urEndEffectorTask_ =
      std::make_shared<mc_tasks::EndEffectorTask>(urBody, robots(), 0);
  urEndEffectorTask_->positionTask->stiffness(1);
  urEndEffectorTask_->orientationTask->stiffness(1);
  urEndEffectorTask_->selectUnactiveJoints(solver(), urJoints_);
  solver().addTask(urEndEffectorTask_);
  kinovaPostureTask_ = std::make_shared<mc_tasks::PostureTask>(solver(), 1);
  solver().addTask(kinovaPostureTask_);
  robots().robot(1).posW(
      sva::PTransformd(sva::RotZ(0.0), Eigen::Vector3d(0.7, 0.5, 0)));
}

CONTROLLER_CONSTRUCTOR("DualArmController", DualArmController)
