#ifndef _H_MCTASKSEFTASK_H_
#define _H_MCTASKSEFTASK_H_

#include <mc_tasks/MetaTask.h>

#include <mc_rbdyn/robot.h>
#include <Tasks/QPTasks.h>

#include <mc_tasks/api.h>

namespace mc_tasks
{

struct MC_TASKS_DLLAPI EndEffectorTask : public MetaTask
{
public:
  EndEffectorTask(const std::string & bodyName, const mc_rbdyn::Robots & robots, unsigned int robotIndex, double stiffness = 10.0, double weight = 1000.0);

  virtual void resetTask(const mc_rbdyn::Robots & robots, unsigned int robotIndex);

  virtual void removeFromSolver(tasks::qp::QPSolver & solver) override;

  virtual void addToSolver(tasks::qp::QPSolver & solver) override;

  virtual void update() override;

  virtual void add_ef_pose(const sva::PTransformd & dtr);

  virtual void set_ef_pose(const sva::PTransformd & tf);

  virtual sva::PTransformd get_ef_pose();

  int dim();

  const Eigen::VectorXd& eval();

  const Eigen::VectorXd& speed();

public:
  const mc_rbdyn::Robots & robots;
  unsigned int robotIndex;

  std::shared_ptr<tasks::qp::PositionTask> positionTask;
  std::shared_ptr<tasks::qp::SetPointTask> positionTaskSp;
  std::shared_ptr<tasks::qp::OrientationTask> orientationTask;
  std::shared_ptr<tasks::qp::SetPointTask> orientationTaskSp;

  std::string bodyName;
  sva::PTransformd curTransform;
  bool inSolver;
  Eigen::VectorXd err;
  Eigen::VectorXd spd;
};

}

#endif
