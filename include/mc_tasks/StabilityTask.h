#ifndef _H_STABILITYTASK_H_
#define _H_STABILITYTASK_H_

#include <mc_tasks/MetaTask.h>
#include <mc_tasks/SmoothTask.h>

#include <mc_rbdyn/robot.h>
#include <mc_rbdyn/stance.h>
#include <mc_rbdyn/StanceConfig.h>

#include <Tasks/QPTasks.h>

#include <mc_tasks/api.h>

namespace mc_tasks
{

struct MC_TASKS_DLLAPI StabilityTask : public MetaTask
{
public:
  StabilityTask(mc_rbdyn::Robots & robots);

  void target(const mc_rbdyn::Robot & env, const mc_rbdyn::Stance & stance,
              const mc_rbdyn::StanceConfig & config, double comSmoothPercent = 1);

  virtual void addToSolver(mc_solver::QPSolver & solver) override;

  virtual void removeFromSolver(mc_solver::QPSolver & solver) override;

  virtual void update() override;

  void highStiffness(const std::vector<std::string> & stiffJoints);

  void normalStiffness(const std::vector<std::string> & stiffJoints);
public:
  mc_rbdyn::Robots & robots;
  mc_rbdyn::Robot & robot;

  /* CoMTask */
  double comStiff;
  double extraComStiff;
  Eigen::Vector3d comObj;
  std::shared_ptr<tasks::qp::CoMTask> comTask;
  std::shared_ptr<tasks::qp::SetPointTask> comTaskSp;
  SmoothTask<Eigen::Vector3d> comTaskSm;

  std::vector< std::vector<double> > qObj;
  std::shared_ptr<tasks::qp::PostureTask> postureTask;
};

}

#endif
