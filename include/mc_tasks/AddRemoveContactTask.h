#ifndef _H_ADDREMOVECONTACTTASK_H_
#define _H_ADDREMOVECONTACTTASK_H_

#include <mc_tasks/MetaTask.h>

#include <mc_rbdyn/contact.h>
#include <mc_rbdyn/robot.h>
#include <mc_rbdyn/StanceConfig.h>

#include <Tasks/QPConstr.h>
#include <Tasks/QPTasks.h>

namespace mc_tasks
{

struct AddRemoveContactTask : public MetaTask
{
public:
  AddRemoveContactTask(mc_rbdyn::Robots & robots, std::shared_ptr<tasks::qp::BoundedSpeedConstr> constSpeedConstr, mc_rbdyn::Contact & contact,
                       double direction, const mc_rbdyn::StanceConfig & config,
                       Eigen::Vector3d * userT_0_s = 0);

  void direction(double direction);

  Eigen::Vector3d velError();

  virtual void addToSolver(tasks::qp::QPSolver & solver) override;

  virtual void removeFromSolver(tasks::qp::QPSolver & solver) override;

  virtual void update() override;
public:
  mc_rbdyn::Robots & robots;
  mc_rbdyn::Robot & robot;
  mc_rbdyn::Robot & env;
  std::shared_ptr<tasks::qp::BoundedSpeedConstr> constSpeedConstr;

  std::shared_ptr<mc_rbdyn::Surface> robotSurf;
  unsigned int robotBodyIndex;
  unsigned int robotBodyId;

  sva::PTransformd targetTf;

  unsigned int bodyId;
  Eigen::MatrixXd dofMat;
  Eigen::MatrixXd speedMat;
  Eigen::Vector3d normal;

  double speed;
  Eigen::Vector3d targetSpeed;
  std::shared_ptr<tasks::qp::LinVelocityTask> linVelTask;
  std::shared_ptr<tasks::qp::PIDTask> linVelTaskPid;
  double targetVelWeight;
};

}

#endif
