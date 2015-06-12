#ifndef _H_MCTASKCOMTASK_H_
#define _H_MCTASKCOMTASK_H_

#include <mc_rbdyn/robot.h>
#include <Tasks/QPSolver.h>
#include <Tasks/QPTasks.h>

namespace mc_tasks
{

struct CoMTask
{
public:
  CoMTask(const mc_rbdyn::Robots & robots, unsigned int robotIndex);

  void resetTask(const mc_rbdyn::Robots & robots, unsigned int robotIndex);

  void removeFromSolver(tasks::qp::QPSolver & solver);

  void addToSolver(tasks::qp::QPSolver & solver);

  void move_com(const Eigen::Vector3d & com);

  void set_com(const Eigen::Vector3d & com);

  Eigen::Vector3d get_com();
public:
  const mc_rbdyn::Robots & robots;

  std::shared_ptr<tasks::qp::CoMTask> comTask;
  std::shared_ptr<tasks::qp::SetPointTask> comTaskSp;

  Eigen::Vector3d cur_com;
  bool in_solver;
};

}

#endif
