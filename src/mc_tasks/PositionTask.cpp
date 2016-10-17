#include <mc_tasks/PositionTask.h>

namespace mc_tasks
{

PositionTask::PositionTask(const std::string & bodyName, const mc_rbdyn::Robots & robots, unsigned int robotIndex, double stiffness, double weight)
: bodyName(bodyName), robots(robots),
  rIndex(robotIndex), bIndex(0),
  inSolver(false)
{
  const mc_rbdyn::Robot & robot = robots.robot(rIndex);
  bIndex = robot.bodyIndexByName(bodyName);

  Eigen::Vector3d curPos = robot.mbc().bodyPosW[bIndex].translation();
  positionTask.reset(new tasks::qp::PositionTask(robots.mbs(), static_cast<int>(rIndex), bodyName, curPos));
  positionTaskSp.reset(new tasks::qp::SetPointTask(robots.mbs(), static_cast<int>(robotIndex), positionTask.get(), stiffness, weight));
}

void PositionTask::resetTask()
{
  const auto & robot = robots.robot(rIndex);
  Eigen::Vector3d curPos = robot.mbc().bodyPosW[bIndex].translation();
  positionTask->position(curPos);
}

void PositionTask::removeFromSolver(mc_solver::QPSolver & solver)
{
  if(inSolver)
  {
    solver.removeTask(positionTaskSp.get());
    inSolver = false;
  }
}

void PositionTask::addToSolver(mc_solver::QPSolver & solver)
{
  if(!inSolver)
  {
    solver.addTask(positionTaskSp.get());
    inSolver = true;
  }
}

void PositionTask::update()
{
}

Eigen::Vector3d PositionTask::position()
{
  return positionTask->position();
}

void PositionTask::position(const Eigen::Vector3d & pos)
{
  positionTask->position(pos);
}

}
