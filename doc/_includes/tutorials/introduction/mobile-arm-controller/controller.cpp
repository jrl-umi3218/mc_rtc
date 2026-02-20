#include "MobileArmController.h"

#include <mc_rbdyn/RobotLoader.h>
#include <thread>
#include <chrono>

MobileArmController::MobileArmController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config) 
: mc_control::MCController(
  {
    rm, 
    mc_rbdyn::RobotLoader::get_robot_module("dingo"),
    mc_rbdyn::RobotLoader::get_robot_module("env/door"),
    mc_rbdyn::RobotLoader::get_robot_module("env/ground")
  }, 
  dt) 
{
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(selfCollisionConstraint);
  solver().addTask(postureTask);
  solver().setContacts({{}});

  Eigen::Vector6d dof = Eigen::Vector6d::Ones();
  double friction = mc_rbdyn::Contact::defaultFriction;
  dof[2] = 0.0;
  dof[3] = 0.0;
  dof[4] = 0.0;
  addContact({"dingo", "ground", "Base", "AllGround", friction, dof});
  addContact({"dingo", "ur5e", "Base", "Base"});

  double iDist = 0.1;
  double sDist = 0.05;
  addCollisions("dingo", "door", {{"*", "*", iDist, sDist, 0}});
  addCollisions("ur5e", "door", {{"*", "*", iDist, sDist, 0}});

  mc_rtc::log::success("MobileArmController init done ");
}

bool MobileArmController::run() 
{
  if (phase_ == APPROACH && count < 1000) 
  {
    count++;
  } 
  else if (phase_ == APPROACH && dingoEndEffectorTask_->eval().norm() < 1e-5 
    && dingoEndEffectorTask_->speed().norm() < 1e-5) 
  { 
    solver().removeTask(postureTask.get());
    handTask_->reset();
    solver().addTask(handTask_);
    handTask_->target(sva::PTransformd(Eigen::Vector3d(0, 0, -0.05)) *
                  robots().robot(2).surfacePose("Handle"));
    phase_ = HANDLE;
  } 
  else if (phase_ == HANDLE && handTask_->eval().norm() < 0.1 
  && handTask_->speed().norm() < 1e-4) 
  {
    addContact({"ur5e", "door", "Tool", "Handle"});
    solver().removeTask(handTask_);
    postureTask->reset();
    doorPostureTask_->target({{"handle", {-1.0}}});
    phase_ = OPEN;
  } 
  else if (phase_ == OPEN && doorPostureTask_->eval().norm() < 0.01) 
  {
    solver().removeTask(dingoEndEffectorTask_);
    doorPostureTask_->target({{"door", {1.57}}});
    phase_ = DONE;
  } 
  else if (phase_ == DONE && doorPostureTask_->eval().norm() < 0.01) 
  {
    removeContact({"ur5e", "door", "Tool", "Handle"});
    dingoEndEffectorTask_->reset();
    solver().addTask(dingoEndEffectorTask_);
    solver().addTask(postureTask.get());
  }
  return mc_control::MCController::run();
}

void MobileArmController::reset(const mc_control::ControllerResetData &reset_data) 
{
  mc_control::MCController::reset(reset_data);
  robots().robot(0).posW(
    sva::PTransformd(sva::RotZ(0.0), Eigen::Vector3d(0.0, 0.0, 0.5)));
  robots().robot(1).posW(
    sva::PTransformd(sva::RotZ(0.0), Eigen::Vector3d(-0.25, 0.0, 0.0)));
  robots().robot(2).posW(
    sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(2.0, 1.0, 0)));

  handTask_ = 
    std::make_shared<mc_tasks::SurfaceTransformTask>("Tool", robots(), 0);
  dingoEndEffectorTask_ =
    std::make_shared<mc_tasks::EndEffectorTask>("base_link", robots(), 1);
  dingoEndEffectorTask_->positionTask->stiffness(1.0);
  dingoEndEffectorTask_->positionTask->weight(1000.0);
  dingoEndEffectorTask_->orientationTask->stiffness(1.0);
  dingoEndEffectorTask_->orientationTask->weight(1000.0);

  doorKinematics_ = 
    std::make_shared<mc_solver::KinematicsConstraint>(robots(), 2, solver().dt());
  solver().addConstraintSet(*doorKinematics_);
  doorPostureTask_ =
    std::make_shared<mc_tasks::PostureTask>(solver(), 2, 1.0, 1.0);
  solver().addTask(doorPostureTask_);

  solver().addTask(dingoEndEffectorTask_);
  solver().addTask(postureTask.get());
  postureTask->target({{"shoulder_lift_joint", {-M_PI / 2}}});
  dingoEndEffectorTask_->add_ef_pose({Eigen::Vector3d(1.5, 0.0, 0.0)});
}

CONTROLLER_CONSTRUCTOR("MobileArmController", MobileArmController)
