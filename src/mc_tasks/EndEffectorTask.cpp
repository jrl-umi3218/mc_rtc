#include <mc_tasks/EndEffectorTask.h>

namespace mc_tasks
{

EndEffectorTask::EndEffectorTask(const std::string & bodyName, const mc_rbdyn::Robots & robots, unsigned int robotIndex, double stiffness, double weight)
: robots(robots), robotIndex(robotIndex),
  bodyName(bodyName)
{
  const mc_rbdyn::Robot & robot = robots.robot(robotIndex);
  bodyIndex = robot.bodyIndexByName(bodyName);
  sva::PTransformd bpw = robot.mbc().bodyPosW[bodyIndex];

  curTransform = bpw;

  positionTask.reset(new mc_tasks::PositionTask(bodyName, robots,
                                                robotIndex, stiffness,
                                                weight));
  orientationTask.reset(new mc_tasks::OrientationTask(bodyName, robots,
                                                      robotIndex, stiffness,
                                                      weight));
}

void EndEffectorTask::reset()
{
  const mc_rbdyn::Robot & robot = robots.robot(robotIndex);
  curTransform = robot.mbc().bodyPosW[bodyIndex];
  positionTask->position(curTransform.translation());
  orientationTask->orientation(curTransform.rotation());
}

void EndEffectorTask::removeFromSolver(mc_solver::QPSolver & solver)
{
  positionTask->removeFromSolver(solver);
  orientationTask->removeFromSolver(solver);
}

void EndEffectorTask::addToSolver(mc_solver::QPSolver & solver)
{
  positionTask->addToSolver(solver);
  orientationTask->addToSolver(solver);
}

void EndEffectorTask::update()
{
}

void EndEffectorTask::add_ef_pose(const sva::PTransformd & dtr)
{
  auto new_rot = curTransform.rotation()*dtr.rotation();
  Eigen::Vector3d new_t = curTransform.translation() + dtr.translation();
  curTransform = sva::PTransformd(new_rot, new_t);
  positionTask->position(curTransform.translation());
  orientationTask->orientation(curTransform.rotation());
}

void EndEffectorTask::set_ef_pose(const sva::PTransformd & tf)
{
  curTransform = tf;
  positionTask->position(tf.translation());
  orientationTask->orientation(tf.rotation());
}

sva::PTransformd EndEffectorTask::get_ef_pose()
{
  return sva::PTransformd(orientationTask->orientation(), positionTask->position());
}

Eigen::VectorXd EndEffectorTask::eval() const
{
  Eigen::Vector6d err;
  err << orientationTask->eval(), positionTask->eval();
  return err;
}

Eigen::VectorXd EndEffectorTask::speed() const
{
  Eigen::Vector6d spd;
  spd << orientationTask->speed(), positionTask->speed();
  return spd;
}

}
