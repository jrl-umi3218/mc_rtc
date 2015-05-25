#ifndef _H_MCTASKSEFTASK_H_
#define _H_MCTASKSEFTASK_H_

#include <mc_rbdyn/robot.h>
#include <Tasks/QPTasks.h>
#include <Tasks/QPSolver.h>

namespace mc_tasks
{

struct EndEffectorTask
{
public:
  EndEffectorTask(const std::string & bodyName, const mc_rbdyn::Robots & robots, unsigned int robotIndex);

  void resetTask(const mc_rbdyn::Robots & robots, unsigned int robotIndex);

  void removeFromSolver(tasks::qp::QPSolver & solver);

  void addToSolver(tasks::qp::QPSolver & solver);

  void add_ef_pose(const sva::PTransformd & dtr);

  void set_ef_pose(const sva::PTransformd & tf);

  sva::PTransformd get_ef_pose();
public:
  const mc_rbdyn::Robots & robots;

  std::shared_ptr<tasks::qp::PositionTask> positionTask;
  std::shared_ptr<tasks::qp::SetPointTask> positionTaskSp;
  std::shared_ptr<tasks::qp::OrientationTask> orientationTask;
  std::shared_ptr<tasks::qp::SetPointTask> orientationTaskSp;

  std::string bodyName;
  sva::PTransformd curTransform;
  bool inSolver;
};

}

#endif
