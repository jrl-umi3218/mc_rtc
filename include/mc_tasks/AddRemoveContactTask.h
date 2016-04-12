#ifndef _H_ADDREMOVECONTACTTASK_H_
#define _H_ADDREMOVECONTACTTASK_H_

#include <mc_tasks/MetaTask.h>

#include <mc_rbdyn/robot.h>
#include <mc_rbdyn/StanceConfig.h>

#include <mc_solver/BoundedSpeedConstr.h>

#include <Tasks/QPTasks.h>

#include <mc_tasks/api.h>

namespace mc_rbdyn
{
  struct Contact;
}

namespace mc_tasks
{

struct MC_TASKS_DLLAPI AddRemoveContactTask : public MetaTask
{
public:
  AddRemoveContactTask(mc_rbdyn::Robots & robots, std::shared_ptr<mc_solver::BoundedSpeedConstr> constSpeedConstr, mc_rbdyn::Contact & contact,
                       double direction, const mc_rbdyn::StanceConfig & config,
                       Eigen::Vector3d * userT_0_s = 0);

  void direction(double direction);

  Eigen::Vector3d velError();

  virtual void addToSolver(mc_solver::QPSolver & solver) override;

  virtual void removeFromSolver(mc_solver::QPSolver & solver) override;

  virtual void update() override;
public:
  mc_rbdyn::Robots & robots;
  mc_rbdyn::Robot & robot;
  mc_rbdyn::Robot & env;
  std::shared_ptr<mc_solver::BoundedSpeedConstr> constSpeedConstr;

  std::shared_ptr<mc_rbdyn::Surface> robotSurf;
  unsigned int robotBodyIndex;

  sva::PTransformd targetTf;

  std::string bodyId;
  Eigen::MatrixXd dofMat;
  Eigen::VectorXd speedMat;
  Eigen::Vector3d normal;

  double speed;
  Eigen::Vector3d targetSpeed;
  std::shared_ptr<tasks::qp::LinVelocityTask> linVelTask;
  std::shared_ptr<tasks::qp::PIDTask> linVelTaskPid;
  double targetVelWeight;
};


struct MC_TASKS_DLLAPI AddContactTask : public AddRemoveContactTask
{
public:
  AddContactTask(mc_rbdyn::Robots & robots, std::shared_ptr<mc_solver::BoundedSpeedConstr> constSpeedConstr, mc_rbdyn::Contact & contact,
                       const mc_rbdyn::StanceConfig & config,
                       Eigen::Vector3d * userT_0_s = 0);
};

struct MC_TASKS_DLLAPI RemoveContactTask : public AddRemoveContactTask
{
public:
  RemoveContactTask(mc_rbdyn::Robots & robots, std::shared_ptr<mc_solver::BoundedSpeedConstr> constSpeedConstr, mc_rbdyn::Contact & contact,
                       const mc_rbdyn::StanceConfig & config,
                       Eigen::Vector3d * userT_0_s = 0);
};

}

#endif
