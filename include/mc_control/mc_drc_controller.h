#ifndef _H_MCDRCCONTROLLER_H_
#define _H_MCDRCCONTROLLER_H_

#include <mc_rbdyn/robot.h>
#include <mc_control/mc_solver/qpsolver.h>

#include <mc_robots/hrp2_drc.h>
#include <mc_robots/env.h>

#include <Tasks/QPTasks.h>

/* FIXME For now this is a simple posture controller, figure a more extensive architecture to allow switching between multiple modes */

namespace mc_control
{

struct MCController
{
};

struct MCDRCPostureController : public MCController
{
public:
  /* Common stuff */
  MCDRCPostureController();

  bool run();

  const mc_control::QPResultMsg & send(const double & t);
  /* Specific to posture controller */
  bool change_joint(int jid);
  bool change_joint(const std::string & jname);

  bool joint_up();
  bool joint_down();

  bool set_joint_pos(const std::string & jname, const double & pos);
public:
  /* Common stuff */
  bool running;
  double timeStep;
  mc_robots::HRP2DRCGripperRobotModule robot_module;
  mc_robots::GroundRobotModule ground_module;
  mc_solver::ContactConstraint contactConstraint;
  mc_solver::KinematicsConstraint kinematicsConstraint;
  mc_solver::CollisionsConstraint selfCollisionConstraint;
  std::shared_ptr<tasks::qp::PostureTask> postureTask;
  std::shared_ptr<mc_solver::QPSolver> qpsolver;
  /* Specific to posture controller */
  int current_joint;
private:
  void add_joint_pos(const double & v);
};

}

#endif
