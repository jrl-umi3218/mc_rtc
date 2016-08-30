#include <mc_tasks/CoMTask.h>

namespace mc_tasks
{

CoMTask::CoMTask(const mc_rbdyn::Robots & robots, unsigned int robotIndex,
                 double stiffness, double weight)
: robots(robots), in_solver(false)
{
  const mc_rbdyn::Robot & robot = robots.robot(robotIndex);

  cur_com = rbd::computeCoM(robot.mb(), robot.mbc());

  comTask.reset(new tasks::qp::CoMTask(robots.mbs(), static_cast<int>(robotIndex), cur_com));
  comTaskSp.reset(new tasks::qp::SetPointTask(robots.mbs(), static_cast<int>(robotIndex), comTask.get(), stiffness, weight));
}

void CoMTask::resetTask(const mc_rbdyn::Robots & robots, unsigned int robotIndex)
{
  const mc_rbdyn::Robot & robot = robots.robot(robotIndex);

  cur_com = rbd::computeCoM(robot.mb(), robot.mbc());

  comTask->com(cur_com);
}

void CoMTask::removeFromSolver(mc_solver::QPSolver & solver)
{
  if(in_solver)
  {
    solver.removeTask(comTaskSp.get());
    in_solver = false;
  }
}

void CoMTask::addToSolver(mc_solver::QPSolver & solver)
{
  if(!in_solver)
  {
    solver.addTask(comTaskSp.get());
    in_solver = true;
  }
}

void CoMTask::update()
{
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

Eigen::Vector3d CoMTask::get_com()
{
  return comTask->com();
}

}
