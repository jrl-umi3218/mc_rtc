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

#include <mc_control/api.h>

namespace mc_control
{

struct MC_CONTROL_DLLAPI MCController : public MCVirtualController
{
public:
  /* Assumes an environment from mc_env_description */
  MCController(double dt, const std::string & env_name = "ground");

  /* Assumes an environment similar to those in mc_env_description but hosted somewhere */
  MCController(double dt, const std::string & env_path, const std::string & env_name);

  /* Generic module for the environment */
  MCController(double dt, const std::shared_ptr<mc_rbdyn::RobotModule> & env);

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
  /* Common services */
  virtual bool joint_up(const std::string & jname) override;
  virtual bool joint_down(const std::string & jname) override;

  virtual bool set_joint_pos(const std::string & jname, const double & pos) override;
public:
  /* Common stuff */
  std::vector< std::pair<Eigen::Vector3d, Eigen::Vector3d> > wrenches;
  mc_robots::HRP2DRCGripperRobotModule robot_module;
  std::shared_ptr<mc_rbdyn::RobotModule> env_module;
  mc_solver::ContactConstraint contactConstraint;
  mc_solver::DynamicsConstraint dynamicsConstraint;
  mc_solver::KinematicsConstraint kinematicsConstraint;
  mc_solver::CollisionsConstraint selfCollisionConstraint;
  std::shared_ptr<tasks::qp::PostureTask> postureTask;
  std::shared_ptr<mc_solver::QPSolver> qpsolver;
};

}

#endif
