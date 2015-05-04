#ifndef _H_MCCONTROLLER_H_
#define _H_MCCONTROLLER_H_

#include <mc_rbdyn/robot.h>
#include <mc_control/mc_solver/qpsolver.h>
#include <mc_control/generic_gripper.h>

#include <mc_robots/hrp2_drc.h>
#include <mc_robots/env.h>

#include <Tasks/QPTasks.h>
#include <mc_tasks/EndEffectorTask.h>

namespace mc_control
{

/* This structure will be filled in time with the necessary information */
/* Coming most likely from the previous controller */
struct ControllerResetData
{
  const std::vector< std::vector<double> > & q;
  const std::vector<mc_rbdyn::Contact> & contacts;
};

struct MCDRCGlobalController;

/*FIXME Get some data as parameters (e.g. timeStep, path to default env...) */
struct MCController
{
  friend struct MCDRCGlobalController;
public:
  MCController(const std::string & env_path = "/home/gergondet/devel-src/mcp/mcp_ws/src/mc_ros/mc_env_description", const std::string & env_name = "ground");

  virtual bool run();

  virtual const mc_control::QPResultMsg & send(const double & t);

  virtual void reset(const ControllerResetData & reset_data);

  /* Helper function to access the robot and the env */
  const mc_rbdyn::Robot & robot() const;

  const mc_rbdyn::Robot & env() const;
public:
  /* Common stuff */
  const double timeStep;
  mc_robots::HRP2DRCGripperRobotModule robot_module;
  mc_robots::EnvRobotModule env_module;
  mc_solver::ContactConstraint contactConstraint;
  mc_solver::DynamicsConstraint dynamicsConstraint;
  mc_solver::KinematicsConstraint kinematicsConstraint;
  mc_solver::CollisionsConstraint selfCollisionConstraint;
  std::shared_ptr<tasks::qp::PostureTask> postureTask;
  std::shared_ptr<mc_solver::QPSolver> qpsolver;
  std::shared_ptr<mc_control::Gripper> lgripper;
  std::shared_ptr<mc_control::Gripper> rgripper;
protected:
  const mc_rbdyn::Robots & robots() const;
  mc_rbdyn::Robots & robots();
  mc_rbdyn::Robot & robot();
  mc_rbdyn::Robot & env();
};

}

#endif
