#include <mc_tasks/CoMTask.h>

namespace mc_tasks
{

CoMTask::CoMTask(const mc_rbdyn::Robots & robots, unsigned int robotIndex)
: robots(robots), in_solver(false)
{
  const mc_rbdyn::Robot & robot = robots.robots[robotIndex];

  cur_com = rbd::computeCoM(*(robot.mb), *(robot.mbc));

  comTask.reset(new tasks::qp::CoMTask(robots.mbs, robotIndex, cur_com));
  comTaskSp.reset(new tasks::qp::SetPointTask(robots.mbs, robotIndex, comTask.get(), 5, 100));
}

void CoMTask::resetTask(const mc_rbdyn::Robots & robots, unsigned int robotIndex)
{
  const mc_rbdyn::Robot & robot = robots.robots[robotIndex];

  cur_com = rbd::computeCoM(*(robot.mb), *(robot.mbc));

  comTask->com(cur_com);
}

void CoMTask::removeFromSolver(tasks::qp::QPSolver & solver)
{
  if(in_solver)
  {
    solver.removeTask(comTaskSp.get());
    solver.updateConstrsNrVars(robots.mbs);
    solver.updateConstrSize();
    in_solver = false;
  }
}

void CoMTask::addToSolver(tasks::qp::QPSolver & solver)
{
  if(!in_solver)
  {
    solver.addTask(comTaskSp.get());
    solver.updateTasksNrVars(robots.mbs);
    solver.updateConstrsNrVars(robots.mbs);
    solver.updateConstrSize();
    in_solver = true;
  }
}

void CoMTask::move_com(const Eigen::Vector3d & com)
{
  cur_com += com;
  comTask->com(cur_com);
}

void CoMTask::set_com(const Eigen::Vector3d & com)
{
  comTask->com(com);
}

}
