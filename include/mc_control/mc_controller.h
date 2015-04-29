#ifndef _H_MCCONTROLLER_H_
#define _H_MCCONTROLLER_H_

#include <mc_rbdyn/robot.h>
#include <mc_control/mc_solver/qpsolver.h>

#include <mc_robots/hrp2_drc.h>
#include <mc_robots/env.h>

#include <Tasks/QPTasks.h>
#include <mc_tasks/EndEffectorTask.h>

namespace mc_control
{

struct MCDRCGlobalController;

/*FIXME Get some data as parameters (e.g. timeStep) */
struct MCController
{
  friend struct MCDRCGlobalController;
public:
  MCController();

  virtual bool run();

  virtual const mc_control::QPResultMsg & send(const double & t);

  virtual void reset(const std::vector< std::vector<double> > & q);

  /* Helper function to access the robot and the env */
  const mc_rbdyn::Robot & robot() const;

  const mc_rbdyn::Robot & env() const;
public:
  /* Common stuff */
  const double timeStep;
  mc_robots::HRP2DRCGripperRobotModule robot_module;
  mc_robots::GroundRobotModule ground_module;
  mc_solver::ContactConstraint contactConstraint;
  mc_solver::DynamicsConstraint dynamicsConstraint;
  mc_solver::KinematicsConstraint kinematicsConstraint;
  mc_solver::CollisionsConstraint selfCollisionConstraint;
  std::shared_ptr<tasks::qp::PostureTask> postureTask;
  std::shared_ptr<mc_solver::QPSolver> qpsolver;
protected:
  mc_rbdyn::Robot & robot();
  mc_rbdyn::Robot & env();
};

}

#endif
