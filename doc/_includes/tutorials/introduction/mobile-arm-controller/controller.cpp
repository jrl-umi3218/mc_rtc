#include "MobileArmController.h"

#include <mc_rbdyn/RobotLoader.h>
#include <chrono>
#include <thread>

MobileArmController::MobileArmController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(
      {rm, mc_rbdyn::RobotLoader::get_robot_module("object", "/usr/local/share/mc_dingo", std::string("dingo")),
       mc_rbdyn::RobotLoader::get_robot_module("env",
                                               std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH)
                                                   + "/../mc_int_obj_description",
                                               std::string("door")),
       mc_rbdyn::RobotLoader::get_robot_module("env",
                                               std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH),
                                               std::string("ground"))},
      dt)
{
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(selfCollisionConstraint);
  Eigen::Vector6d dof = Eigen::Vector6d::Ones();
  double friction = mc_rbdyn::Contact::defaultFriction;
  dof[2] = 0.0;
  dof[3] = 0.0;
  dof[4] = 0.0;
  addContact({"dingo", "ground", "Base", "AllGround", friction, dof});
  double iDist = 0.1;
  double sDist = 0.05;
  addCollisions("dingo", "door", {{"*", "*", iDist, sDist, 0}});
  addCollisions("ur5e", "door", {{"*", "*", iDist, sDist, 0}});
  mc_rtc::log::success("MobileArmController init done ");
}

bool MobileArmController::run()
{
  if(phase == APPROACH && count < 1000) { count++; }
  else if(phase == APPROACH && dingoEndEffectorTask_->eval().norm() < 1e-5
          && dingoEndEffectorTask_->speed().norm() < 1e-5)
  {
    solver().removeTask(postureTask.get());
    handTask_->reset();
    solver().addTask(handTask_);
    handTask_->target(sva::PTransformd(Eigen::Vector3d(0, 0, -0.05)) * robots().robot(2).surfacePose("Handle"));
    phase = HANDLE;
  }
  else if(phase == HANDLE && handTask_->eval().norm() < 0.1 && handTask_->speed().norm() < 1e-4)
  {
    addContact({"ur5e", "door", "Wrist", "Handle"});
    solver().removeTask(handTask_);
    postureTask->reset();
    doorPostureTask_->target({{"handle", {-1.0}}});
    phase = OPEN;
  }
  else if(phase == OPEN && doorPostureTask_->eval().norm() < 0.01)
  {
    solver().removeTask(dingoEndEffectorTask_);
    doorPostureTask_->target({{"door", {1.57}}});
    phase = DONE;
  }
  else if(phase == DONE && doorPostureTask_->eval().norm() < 0.01)
  {
    removeContact({"ur5e", "door", "Wrist", "Handle"});
    dingoEndEffectorTask_->reset();
    solver().addTask(dingoEndEffectorTask_);
    solver().addTask(postureTask.get());
  }
  return mc_control::MCController::run();
}

void MobileArmController::reset(const mc_control::ControllerResetData & reset_data)
{
  doorKinematics_ = std::make_shared<mc_solver::KinematicsConstraint>(robots(), 2, solver().dt());
  solver().addConstraintSet(*doorKinematics_);
  mc_control::MCController::reset(reset_data);
  robots().robot(0).posW(sva::PTransformd(sva::RotZ(0.0), Eigen::Vector3d(0.0, 0.0, 0.5)));
  robots().robot(1).posW(sva::PTransformd(sva::RotZ(0.0), Eigen::Vector3d(0.0, 0.0, 0.0)));
  robots().robot(2).posW(sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(2.0, 1.0, 0)));
  addContact({"ur5e", "dingo", "Base", "Base"});
  doorPostureTask_ = std::make_shared<mc_tasks::PostureTask>(solver(), 2, 1.0, 1.0);
  solver().addTask(doorPostureTask_);
  handTask_ = std::make_shared<mc_tasks::SurfaceTransformTask>("Wrist", robots(), 0);
  dingoEndEffectorTask_ = std::make_shared<mc_tasks::EndEffectorTask>("base_link", robots(), 1);
  dingoEndEffectorTask_->positionTask->stiffness(1.0);
  dingoEndEffectorTask_->positionTask->weight(1000.0);
  dingoEndEffectorTask_->orientationTask->stiffness(1.0);
  dingoEndEffectorTask_->orientationTask->weight(1000.0);
  solver().addTask(dingoEndEffectorTask_);
  solver().addTask(postureTask.get());
  postureTask->target({{"shoulder_lift_joint", {-M_PI / 2}}});
  dingoEndEffectorTask_->add_ef_pose({Eigen::Vector3d(1.5, 0.0, 0.0)});
  count = 0;
}

CONTROLLER_CONSTRUCTOR("MobileArmController", MobileArmController)
