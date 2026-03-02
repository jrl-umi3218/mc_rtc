#include "MobileArmController.h"

#include <mc_rbdyn/RobotLoader.h>
#include <mc_tasks/TransformTask.h>
#include <chrono>
#include <thread>

MobileArmController::MobileArmController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController({rm, mc_rbdyn::RobotLoader::get_robot_module("dingo"),
                            mc_rbdyn::RobotLoader::get_robot_module("env/door"),
                            mc_rbdyn::RobotLoader::get_robot_module("env/ground")},
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
  if(phase_ == APPROACH && dingoBaseTask_->eval().norm() < 1e-2 && dingoBaseTask_->speed().norm() < 1e-3)
  {
    solver().removeTask(postureTask);
    handTask_->reset();
    solver().addTask(handTask_);
    handTask_->target(sva::PTransformd(Eigen::Vector3d(0, 0, -0.05)) * robots().robot(2).surfacePose("Handle"));
    phase_ = HANDLE;
  }
  else if(phase_ == HANDLE && handTask_->eval().norm() < 0.1 && handTask_->speed().norm() < 1e-4)
  {
    addContact({"ur5e", "door", "Tool", "Handle"});
    solver().removeTask(handTask_);
    doorPostureTask_->target({{"handle", {-1.0}}});
    phase_ = OPEN;
  }
  else if(phase_ == OPEN && doorPostureTask_->eval().norm() < 0.01)
  {
    solver().removeTask(dingoBaseTask_);
    doorPostureTask_->target({{"door", {M_PI / 2}}});
    phase_ = DONE;
  }
  else if(phase_ == DONE && doorPostureTask_->eval().norm() < 0.01)
  {
    removeContact({"ur5e", "door", "Tool", "Handle"});
    solver().addTask(dingoBaseTask_);
    solver().addTask(postureTask);
  }
  return mc_control::MCController::run();
}

void MobileArmController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);
  robots().robot(0).posW(sva::PTransformd(sva::RotZ(0.0), Eigen::Vector3d(0.0, 0.0, 0.5)));
  robots().robot(1).posW(sva::PTransformd(sva::RotZ(0.0), Eigen::Vector3d(-0.25, 0.0, 0.0)));
  robots().robot(2).posW(sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(2.0, 1.0, 0)));

  handTask_ = std::make_shared<mc_tasks::SurfaceTransformTask>("Tool", robots(), 0);
  dingoBaseTask_ = std::make_shared<mc_tasks::TransformTask>("base_link", robots(), 1, 2.0, 1000);

  doorKinematics_ = std::make_shared<mc_solver::KinematicsConstraint>(robots(), 2, solver().dt());
  solver().addConstraintSet(*doorKinematics_);
  doorPostureTask_ = std::make_shared<mc_tasks::PostureTask>(solver(), 2, 1.0, 1.0);
  solver().addTask(doorPostureTask_);

  solver().addTask(dingoBaseTask_);
  postureTask->target({{"shoulder_lift_joint", {-M_PI / 2}}});
  dingoBaseTask_->target({Eigen::Vector3d(1.5, 0.0, 0.0)});
}

CONTROLLER_CONSTRUCTOR("MobileArmController", MobileArmController)
