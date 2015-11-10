#include <mc_tasks/RelativeEndEffectorTask.h>

namespace mc_tasks
{

RelativeEndEffectorTask::RelativeEndEffectorTask(const std::string & bodyName, const mc_rbdyn::Robots & robots, unsigned int robotIndex, unsigned int relBodyIdx, double stiffness, double weight)
: EndEffectorTask(bodyName, robots, robotIndex, stiffness, weight),
  relBodyIdx(relBodyIdx)
{
  resetTask(robots, robotIndex);
}

void RelativeEndEffectorTask::resetTask(const mc_rbdyn::Robots & robots, unsigned int robotIndex)
{
  const mc_rbdyn::Robot & robot = robots.robot(robotIndex);
  unsigned int bodyIndex = robot.bodyIndexByName(bodyName);
  sva::PTransformd X_0_body = robot.mbc().bodyPosW[bodyIndex];
  sva::PTransformd X_0_rel = robot.mbc().bodyPosW[relBodyIdx];

  curTransform = X_0_body * (X_0_rel.inv()); /* X_rel_body = X_0_body * X_rel_0 */
}

void RelativeEndEffectorTask::add_ef_pose(const sva::PTransformd & dtr)
{
  auto new_rot = curTransform.rotation()*dtr.rotation();
  Eigen::Vector3d new_t = curTransform.translation() + dtr.translation();
  curTransform = sva::PTransformd(new_rot, new_t);
}

void RelativeEndEffectorTask::set_ef_pose(const sva::PTransformd & tf)
{
  curTransform = tf;
}

void RelativeEndEffectorTask::update()
{
  const sva::PTransformd & X_0_rel = robots.robot(robotIndex).mbc().bodyPosW[relBodyIdx];
  sva::PTransformd X_0_bodyDes = curTransform * X_0_rel; /* X_0_body = X_rel_body * X_0_rel */
  positionTask->position(X_0_bodyDes.translation());
  orientationTask->orientation(X_0_bodyDes.rotation());
}

sva::PTransformd RelativeEndEffectorTask::get_ef_pose()
{
  return curTransform;
}

}
