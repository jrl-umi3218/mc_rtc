#include <mc_tasks/OrientationTask.h>

namespace mc_tasks
{

OrientationTask::OrientationTask(const std::string & bodyName, const mc_rbdyn::Robots & robots, unsigned int robotIndex, double stiffness, double weight)
: TrajectoryTaskGeneric<tasks::qp::OrientationTask>(robots, robotIndex, stiffness, weight),
  bodyName(bodyName), bIndex(0)
{
  const mc_rbdyn::Robot & robot = robots.robot(rIndex);
  bIndex = robot.bodyIndexByName(bodyName);

  Eigen::Matrix3d curOri = robot.mbc().bodyPosW[bIndex].rotation();
  finalize(robots.mbs(), static_cast<int>(rIndex), bodyName, curOri);
}

void OrientationTask::reset()
{
  const auto & robot = robots.robot(rIndex);
  auto curOri = robot.mbc().bodyPosW[bIndex].rotation();
  errorT->orientation(curOri);
}

void OrientationTask::orientation(const Eigen::Matrix3d & ori)
{
  errorT->orientation(ori);
}

Eigen::Matrix3d OrientationTask::orientation()
{
  return errorT->orientation();
}

void OrientationTask::addToLogger(mc_control::Logger & logger)
{
  const auto & robot = robots.robot(rIndex);
  logger.addLogEntry(robot.name() + "_" + bodyName + "_orientation_target",
                     [this]()
                     {
                     return Eigen::Quaterniond(orientation());
                     });
  logger.addLogEntry(robot.name() + "_" + bodyName + "_orientation",
                     [this]()
                     {
                     return Eigen::Quaterniond(robots.robot(rIndex).mbc().bodyPosW[bIndex].rotation());
                     });
}

void OrientationTask::removeFromLogger(mc_control::Logger & logger)
{
  const auto & robot = robots.robot(rIndex);
  logger.removeLogEntry(robot.name() + "_" + bodyName + "_orientation_target");
  logger.removeLogEntry(robot.name() + "_" + bodyName + "_orientation");
}

}
