#ifndef _H_MOVECONTACTTASK_H_
#define _H_MOVECONTACTTASK_H_

#include <mc_tasks/MetaTask.h>
#include <mc_tasks/SmoothTask.h>

#include <mc_rbdyn/robot.h>
#include <mc_rbdyn/StanceConfig.h>

#include <Tasks/QPConstr.h>
#include <Tasks/QPTasks.h>

#include <mc_tasks/api.h>

namespace mc_rbdyn
{
  struct Contact;
}

namespace mc_tasks
{

struct MC_TASKS_DLLAPI MoveContactTask : public MetaTask
{
public:
  MoveContactTask(mc_rbdyn::Robots & robots, mc_rbdyn::Robot & robot, mc_rbdyn::Robot & env, mc_rbdyn::Contact & contact, mc_rbdyn::StanceConfig & config, double positionWStartPercent = 0);

  void toWaypoint(mc_rbdyn::StanceConfig & config, double positionSmoothPercent = 1);

  void toPreEnv(mc_rbdyn::StanceConfig & config, double positionSmoothPercent = 1);

  void toEnv(mc_rbdyn::StanceConfig & config, double positionSmoothPercent = 1);

  void target(const Eigen::Vector3d & pos, const Eigen::Matrix3d & ori, mc_rbdyn::StanceConfig & config, double positionSmoothPercent);

  sva::PTransformd robotSurfacePos();

  sva::MotionVecd robotSurfaceVel();

  virtual void addToSolver(tasks::qp::QPSolver & solver) override;

  virtual void removeFromSolver(tasks::qp::QPSolver & solver) override;

  virtual void update() override;
public:
  mc_rbdyn::Robots & robots;
  mc_rbdyn::Robot & robot;
  mc_rbdyn::Robot & env;

  std::shared_ptr<mc_rbdyn::Surface> robotSurf;
  std::shared_ptr<mc_rbdyn::Surface> envSurf;

  unsigned int robotBodyIndex;
  int robotBodyId;
  unsigned int envBodyIndex;
  int envBodyId;

  sva::PTransformd targetTf;
  Eigen::Vector3d targetPos;
  Eigen::Matrix3d targetOri;

  Eigen::Vector3d normal;
  Eigen::Vector3d preTargetPos;
  Eigen::Vector3d wp;

  double posStiff;
  double extraPosStiff;
  std::shared_ptr<tasks::qp::PositionTask> positionTask;
  std::shared_ptr<tasks::qp::SetPointTask> positionTaskSp;
  std::shared_ptr<SmoothTask<Eigen::Vector3d>> positionTaskSm;

  std::shared_ptr<tasks::qp::OrientationTask> orientationTask;
  std::shared_ptr<tasks::qp::SetPointTask> orientationTaskSp;
  bool useSmoothTask;
};

}

#endif
