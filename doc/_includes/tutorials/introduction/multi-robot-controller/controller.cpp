#include "MyFirstController.h"

#include <mc_rbdyn/RobotLoader.h>

MyFirstController::MyFirstController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController({rm,
                           mc_rbdyn::RobotLoader::get_robot_module("env", std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH) + "/../mc_int_obj_description", std::string("door")),
                           mc_rbdyn::RobotLoader::get_robot_module("env", std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH), std::string("ground"))}, dt)
{
  config_.load(config);
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(dynamicsConstraint);
  solver().addTask(postureTask);
  addContact({robot().name(), "ground", "LeftFoot", "AllGround"});
  addContact({robot().name(), "ground", "RightFoot", "AllGround"});
  comTask = std::make_shared<mc_tasks::CoMTask>(robots(), 0, 10.0, 1000.0);
  solver().addTask(comTask);
  postureTask->stiffness(1);

  mc_rtc::log::success("MyFirstController init done");
}

bool MyFirstController::run()
{
  switch_phase();
  return mc_control::MCController::run();
}

void MyFirstController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);
  comTask->reset();
  robots().robot(1).posW(sva::PTransformd(sva::RotZ(M_PI), Eigen::Vector3d(0.7, 0.5, 0)));
  doorKinematics = std::make_shared<mc_solver::KinematicsConstraint>(robots(), 1, solver().dt());
  solver().addConstraintSet(*doorKinematics);
  doorPosture = std::make_shared<mc_tasks::PostureTask>(solver(), 1, 5.0, 1000.0);
  solver().addTask(doorPosture);
  handTask = std::make_shared<mc_tasks::SurfaceTransformTask>("RightGripper", robots(), 0);
  solver().addTask(handTask);
  handTask->target(sva::PTransformd(Eigen::Vector3d(0, 0, -0.025)) * robots().robot(1).surfacePose("Handle"));
}

void MyFirstController::switch_phase()
{
  if(phase == APPROACH && handTask->eval().norm() < 0.05 && handTask->speed().norm() < 1e-4)
  {
    // Add a new contact
    addContact({robot().name(), "door", "RightGripper", "Handle"});
    // Remove the surface transform task
    solver().removeTask(handTask);
    // Keep the robot in its current posture
    postureTask->reset();
    comTask->reset();
    // Target new handle position
    doorPosture->target({{"handle", {-1.0}}});
    // Switch phase
    phase = HANDLE;
  }
  else if(phase == HANDLE && doorPosture->eval().norm() < 0.01)
  {
    // Update door opening target
    doorPosture->target({{"door", {0.5}}});
    // Switch phase
    phase = OPEN;
  }
}

CONTROLLER_CONSTRUCTOR("MyFirstController", MyFirstController)
