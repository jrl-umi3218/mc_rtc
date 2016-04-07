#include <mc_tasks/OrientationTask.h>

namespace mc_tasks
{

OrientationTask::OrientationTask(const std::string & bodyName, const mc_rbdyn::Robots & robots, unsigned int robotIndex, double stiffness, double weight)
: bodyName(bodyName), robots(robots),
  rIndex(robotIndex), bIndex(0),
  inSolver(false)
{
  const mc_rbdyn::Robot & robot = robots.robot(rIndex);
  bIndex = robot.bodyIndexByName(bodyName);

  Eigen::Matrix3d curOri = robot.mbc().bodyPosW[bIndex].rotation();
  orientationTask.reset(new tasks::qp::OrientationTask(robots.mbs(), static_cast<int>(rIndex), bodyName, curOri));
  orientationTaskSp.reset(new tasks::qp::SetPointTask(robots.mbs(), static_cast<int>(robotIndex), orientationTask.get(), stiffness, weight));
}

void OrientationTask::resetTask()
{
  const auto & robot = robots.robot(rIndex);
  auto curOri = robot.mbc().bodyPosW[bIndex].rotation();
  orientationTask->orientation(curOri);
}

void OrientationTask::removeFromSolver(mc_solver::QPSolver & solver)
{
  if(inSolver)
  {
    solver.removeTask(orientationTaskSp.get());
    inSolver = false;
  }
}

void OrientationTask::addToSolver(mc_solver::QPSolver & solver)
{
  if(!inSolver)
  {
    solver.addTask(orientationTaskSp.get());
    inSolver = true;
  }
}

void OrientationTask::set_ef_ori(const Eigen::Matrix3d & ori)
{
  orientationTask->orientation(ori);
}

Eigen::Matrix3d OrientationTask::get_ef_ori()
{
  return orientationTask->orientation();
}

}
