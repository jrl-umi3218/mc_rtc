#ifndef _H_MCTASKSORITASK_H_
#define _H_MCTASKSORITASK_H_

#include <mc_rbdyn/robot.h>
#include <mc_solver/qpsolver.h>
#include <Tasks/QPTasks.h>

#include <mc_tasks/api.h>

namespace mc_tasks
{

struct MC_TASKS_DLLAPI OrientationTask
{
public:
  OrientationTask(const std::string & bodyName, const mc_rbdyn::Robots & robots, unsigned int robotIndex, double stiffness = 2.0, double weight = 500);

  void resetTask();

  void removeFromSolver(mc_solver::QPSolver & solver);

  void addToSolver(mc_solver::QPSolver & solver);

  void set_ef_ori(const Eigen::Matrix3d & ori);

  Eigen::Matrix3d get_ef_ori();
public:
  std::string bodyName;
  const mc_rbdyn::Robots & robots;
  unsigned int rIndex;
  int bId;
  unsigned int bIndex;

  bool inSolver;
  std::shared_ptr<tasks::qp::OrientationTask> orientationTask;
  std::shared_ptr<tasks::qp::SetPointTask> orientationTaskSp;
};

}

#endif
