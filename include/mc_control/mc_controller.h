#ifndef _H_MCCONTROLLER_H_
#define _H_MCCONTROLLER_H_

#include <mc_rtc/config.h>

#include <mc_control/mc_virtual_controller.h>

#include <mc_rbdyn/robot.h>
#include <mc_solver/qpsolver.h>
#include <mc_control/generic_gripper.h>

#include <mc_robots/hrp2_drc.h>
#include <mc_robots/env.h>

#include <Tasks/QPTasks.h>
#include <mc_tasks/EndEffectorTask.h>

namespace mc_control
{

/*FIXME Get some data as parameters (e.g. timeStep, path to default env...) */
struct MCController : public MCVirtualController
{
public:
  MCController(const std::string & env_path = mc_rtc::MC_ENV_DESCRIPTION_PATH, const std::string & env_name = "ground");

  virtual bool run() override;

  virtual const QPResultMsg & send(const double & t) override;

  virtual void reset(const ControllerResetData & reset_data) override;

  virtual void setWrenches(const std::vector< std::pair<Eigen::Vector3d, Eigen::Vector3d> > & wrenches) override;
  /* Helper function to access robots, robot and env */
  virtual const mc_rbdyn::Robot & robot() const override;

  virtual const mc_rbdyn::Robot & env() const override;

  virtual const mc_rbdyn::Robots & robots() const override;

  virtual mc_rbdyn::Robots & robots() override;

  virtual mc_rbdyn::Robot & robot() override;

  virtual mc_rbdyn::Robot & env() override;
public:
  /* Common stuff */
  std::vector< std::pair<Eigen::Vector3d, Eigen::Vector3d> > wrenches;
  mc_robots::HRP2DRCGripperRobotModule robot_module;
  mc_robots::EnvRobotModule env_module;
  mc_solver::ContactConstraint contactConstraint;
  mc_solver::DynamicsConstraint dynamicsConstraint;
  mc_solver::KinematicsConstraint kinematicsConstraint;
  mc_solver::CollisionsConstraint selfCollisionConstraint;
  std::shared_ptr<tasks::qp::PostureTask> postureTask;
  std::shared_ptr<mc_solver::QPSolver> qpsolver;
};

}

#endif
