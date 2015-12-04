#ifndef _H_MCMRQPCONTROLLER_H_
#define _H_MCMRQPCONTROLLER_H_

#include <mc_control/mc_virtual_controller.h>

#include <mc_rbdyn/robot.h>
#include <mc_solver/qpsolver.h>
#include <mc_control/generic_gripper.h>

#include <mc_robots/hrp2_drc.h>
#include <mc_robots/env.h>

#include <Tasks/QPTasks.h>

#include <mc_control/api.h>

namespace mc_control
{

struct MC_CONTROL_DLLAPI MCMRQPController : public MCVirtualController
{
public:
  /* FIXME Since this is aimed at controlling HRP-2 only for the moment, we
   * will assume that we only load additional robots and that the main robot is
   * robot 0 */
  MCMRQPController(double dt, const std::vector<std::shared_ptr<mc_rbdyn::RobotModule>> & env_modules);

  virtual bool run() override;

  /*FIXME Not very good/natural but once again the actual aim is to control one
   * robot, the states of other robots is internal */
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
  std::vector<std::shared_ptr<mc_rbdyn::RobotModule>> robot_modules;
  mc_solver::ContactConstraint hrp2contactConstraint;
  mc_solver::DynamicsConstraint hrp2dynamicsConstraint;
  mc_solver::KinematicsConstraint hrp2kinematicsConstraint;
  mc_solver::CollisionsConstraint hrp2selfCollisionConstraint;
  std::shared_ptr<tasks::qp::PostureTask> hrp2postureTask;
  std::shared_ptr<mc_solver::QPSolver> mrqpsolver;
  QPResultMsg qpres;
};

}

#endif
