#ifndef _H_MCTASKCOMTASK_H_
#define _H_MCTASKCOMTASK_H_

#include <mc_rbdyn/robot.h>
#include <mc_control/mc_solver/qpsolver.h>
#include <Tasks/QPTasks.h>

namespace mc_tasks
{

struct CoMTask
{
public:
  CoMTask(const mc_rbdyn::Robots & robots, unsigned int robotIndex);

  void resetTask(const mc_rbdyn::Robots & robots, unsigned int robotIndex);

  void removeFromSolver(mc_solver::QPSolver & solver);

  void addToSolver(mc_solver::QPSolver & solver);

  void move_com(const Eigen::Vector3d & com);

  void set_com(const Eigen::Vector3d & com);
public:
  std::shared_ptr<tasks::qp::CoMTask> comTask;
  std::shared_ptr<tasks::qp::SetPointTask> comTaskSp;

  Eigen::Vector3d cur_com;
};

}

#endif
