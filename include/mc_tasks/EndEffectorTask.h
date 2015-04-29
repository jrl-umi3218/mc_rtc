#ifndef _H_MCTASKSEFTASK_H_
#define _H_MCTASKSEFTASK_H_

#include <mc_rbdyn/robot.h>
#include <mc_control/mc_solver/qpsolver.h>
#include <Tasks/QPTasks.h>

namespace mc_tasks
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

}

#endif
