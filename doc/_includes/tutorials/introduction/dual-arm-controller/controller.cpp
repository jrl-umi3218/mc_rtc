#include "DualArmController.h"

#include <mc_rbdyn/RobotLoader.h>

DualArmController::DualArmController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController({rm, mc_rbdyn::RobotLoader::get_robot_module("KinovaDefault")}, dt)
{
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(kinematicsConstraint);
  solver().addConstraintSet(selfCollisionConstraint);

  addCollisions("ur5e", "kinova_default", {{"*", "*", iDist, sDist, 0}});

  postureTask->stiffness(1);
  postureTask->weight(1);
  solver().addTask(postureTask);

  solver().setContacts({{}});
}

bool DualArmController::run()
{
  if(phase_ == IDLE)
  {
    postureTask->target({{"elbow_joint", {-M_PI / 2}}, {"wrist_2_joint", {M_PI / 2}}});
    phase_ = STARTED;
  }
  else if(phase_ == STARTED && postureTask->eval().norm() < 0.01 && postureTask->speed().norm() < 0.01)
  {
    phase_ = MOVE;
    solver().addTask(urEndEffectorTask_);
  }
  else if(phase_ == STARTED) { urEndEffectorTask_->reset(); }
  else if(phase_ == MOVE)
  {
    runUr();
    runKinova();
  }
  return mc_control::MCController::run();
}

void DualArmController::runUr()
{
  if(urState_ == GO && urEndEffectorTask_->eval().norm() < 0.05 && urEndEffectorTask_->speed().norm() < 0.01)
  {
    urState_ = RETURN;
    urEndEffectorTask_->add_ef_pose({Eigen::Vector3d(0.0, -0.5, 0.0)});
  }
  else if(urState_ == RETURN && urEndEffectorTask_->eval().norm() < 0.01 && urEndEffectorTask_->speed().norm() < 0.05)
  {
    urState_ = GO;
    urEndEffectorTask_->add_ef_pose({Eigen::Vector3d(0.0, 0.5, 0.0)});
  }
}

void DualArmController::runKinova()
{
  if(kinovaState_ == GO && kinovaPostureTask_->eval().norm() < 0.01 && kinovaPostureTask_->speed().norm() < 0.01)
  {
    kinovaState_ = RETURN;
    kinovaPostureTask_->target({{"joint_2", {0.0}}});
  }
  else if(kinovaState_ == RETURN && kinovaPostureTask_->eval().norm() < 0.01
          && kinovaPostureTask_->speed().norm() < 0.01)
  {
    kinovaState_ = GO;
    kinovaPostureTask_->target({{"joint_2", {-M_PI / 4}}});
  }
}

void DualArmController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);

  std::string urBody = "tool0";
  urEndEffectorTask_ = std::make_shared<mc_tasks::EndEffectorTask>(urBody, robots(), 0, 1);

  kinovaPostureTask_ = std::make_shared<mc_tasks::PostureTask>(solver(), 1, 1, 1);
  solver().addTask(kinovaPostureTask_);

  kinovaKinematics_ = std::make_unique<mc_solver::KinematicsConstraint>(robots(), 1, solver().dt());
  solver().addConstraintSet(kinovaKinematics_);

  robots().robot(1).posW(sva::PTransformd(sva::RotZ(0.0), Eigen::Vector3d(0.7, 0.5, 0)));
}

CONTROLLER_CONSTRUCTOR("DualArmController", DualArmController)
