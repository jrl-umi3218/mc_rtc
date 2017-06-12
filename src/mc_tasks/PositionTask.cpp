#include <mc_tasks/PositionTask.h>

namespace mc_tasks
{

PositionTask::PositionTask(const std::string & bodyName, const mc_rbdyn::Robots & robots, unsigned int robotIndex, double stiffness, double weight)
: PositionTask(bodyName, Eigen::Vector3d::Zero(), robots, robotIndex, stiffness, weight)
{
}

PositionTask::PositionTask(const std::string & bodyName, const Eigen::Vector3d& bodyPoint,
    const mc_rbdyn::Robots & robots, unsigned int robotIndex, double stiffness, double weight)
: TrajectoryTaskGeneric<tasks::qp::PositionTask>(robots, robotIndex, stiffness, weight),
  bodyName(bodyName), bIndex(0)
{
  const mc_rbdyn::Robot & robot = robots.robot(rIndex);
  bIndex = robot.bodyIndexByName(bodyName);

  Eigen::Vector3d curPos = robot.mbc().bodyPosW[bIndex].translation();
  finalize(robots.mbs(), static_cast<int>(rIndex), bodyName, curPos, bodyPoint);
}


void PositionTask::reset()
{
  const auto & robot = robots.robot(rIndex);
  Eigen::Vector3d curPos = robot.mbc().bodyPosW[bIndex].translation();
  errorT->position(curPos);
}

Eigen::Vector3d PositionTask::position()
{
  return errorT->position();
}

void PositionTask::position(const Eigen::Vector3d & pos)
{
  errorT->position(pos);
}

Eigen::Vector3d PositionTask::bodyPoint() const
{
  return errorT->bodyPoint();
}

void PositionTask::bodyPoint(const Eigen::Vector3d& bodyPoint)
{
  errorT->bodyPoint(bodyPoint);
}

}
