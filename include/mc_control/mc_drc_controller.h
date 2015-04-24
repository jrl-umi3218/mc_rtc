#ifndef _H_MCDRCCONTROLLER_H_
#define _H_MCDRCCONTROLLER_H_

#include <mc_rbdyn/robot.h>
#include <mc_control/mc_solver/qpsolver.h>

#include <mc_robots/hrp2_drc.h>
#include <mc_robots/env.h>

#include <Tasks/QPTasks.h>

/* FIXME For now, everything is in there, split it would be good */

namespace mc_control
{

struct EndEffectorTask
{
public:
  EndEffectorTask(const std::string & bodyName, const mc_rbdyn::Robots & robots, unsigned int robotIndex);

  void resetTask(const mc_rbdyn::Robots & robots, unsigned int robotIndex);

  void removeFromSolver(mc_solver::QPSolver & solver);

  void addToSolver(mc_solver::QPSolver & solver);

  void add_ef_pose(const sva::PTransformd & dtr);
public:
  std::shared_ptr<tasks::qp::PositionTask> positionTask;
  std::shared_ptr<tasks::qp::SetPointTask> positionTaskSp;
  std::shared_ptr<tasks::qp::OrientationTask> orientationTask;
  std::shared_ptr<tasks::qp::SetPointTask> orientationTaskSp;

  std::string bodyName;
  sva::PTransformd curTransform;
};

/*FIXME Get some data as parameters (e.g. timeStep) */
/*FIXME Give initial q to the MCController */
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
  mc_solver::KinematicsConstraint kinematicsConstraint;
  mc_solver::CollisionsConstraint selfCollisionConstraint;
  std::shared_ptr<tasks::qp::PostureTask> postureTask;
  std::shared_ptr<mc_solver::QPSolver> qpsolver;
protected:
  mc_rbdyn::Robot & robot();
  mc_rbdyn::Robot & env();
};

struct MCDRCPostureController : public MCController
{
public:
  /* Common stuff */
  MCDRCPostureController();

  /* Specific to posture controller */
  bool change_joint(int jid);
  bool change_joint(const std::string & jname);

  bool joint_up();
  bool joint_down();

  bool set_joint_pos(const std::string & jname, const double & pos);
public:
  /* Specific to posture controller */
  int current_joint;
private:
  void add_joint_pos(const double & v);
};

struct MCDRCBody6dController : public MCController
{
public:
  MCDRCBody6dController();

  virtual void reset(const std::vector< std::vector<double> > & q) override;

  /* Specific to 6d controller */
  bool change_ef(const std::string & ef_name);

  bool translate_ef(const Eigen::Vector3d & t);

  bool rotate_ef(const Eigen::Matrix3d & m);
public:
  std::shared_ptr<EndEffectorTask> efTask;
};

struct MCDRCGlobalController
{
public:
  MCDRCGlobalController();

  void init(const std::vector<double> & initq);

  bool run();

  const mc_control::QPResultMsg & send(const double & t);

  const mc_solver::QPSolver & qpsolver() const;
  /* Called by the RT component to switch between controllers */
  bool EnablePostureController();
  bool EnableBody6dController();

  /* Called by the RT component to access PostureController service */
  bool change_joint(int jid);
  bool change_joint(const std::string & jname);
  bool joint_up();
  bool joint_down();
  bool set_joint_pos(const std::string & jname, const double & pos);

  /* Called by the RT component to access Body6dController service */
  bool change_ef(const std::string & ef_name);
  bool translate_ef(const Eigen::Vector3d & t);
  bool rotate_ef(const Eigen::Matrix3d & m);
public:
  bool running;
private:
  MCDRCPostureController posture_controller;
  MCDRCBody6dController body6d_controller;
  enum CurrentController
  {
    POSTURE = 1,
    BODY6D  = 2
  };
  CurrentController current_ctrl;
  CurrentController next_ctrl;
  MCController * controller;
  MCController * next_controller;
};

}

#endif
