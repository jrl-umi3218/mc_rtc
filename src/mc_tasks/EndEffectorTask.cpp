#include <mc_tasks/EndEffectorTask.h>

namespace mc_tasks
{

EndEffectorTask::EndEffectorTask(const std::string & bodyName, const mc_rbdyn::Robots & robots, unsigned int robotIndex, double stiffness, double weight)
: EndEffectorTask(bodyName, Eigen::Vector3d::Zero(), robots, robotIndex, stiffness, weight)
{
}

EndEffectorTask::EndEffectorTask(const std::string & bodyName, const Eigen::Vector3d & bodyPoint, const mc_rbdyn::Robots & robots, unsigned int robotIndex, double stiffness, double weight)
: robots(robots), robotIndex(robotIndex),
  bodyName(bodyName)
{
  const mc_rbdyn::Robot & robot = robots.robot(robotIndex);
  bodyIndex = robot.bodyIndexByName(bodyName);
  sva::PTransformd bpw = robot.mbc().bodyPosW[bodyIndex];

  curTransform = bpw;

  positionTask.reset(new mc_tasks::PositionTask(bodyName, bodyPoint, robots,
                                                robotIndex, stiffness,
                                                weight));
  orientationTask.reset(new mc_tasks::OrientationTask(bodyName, robots,
                                                      robotIndex, stiffness,
                                                      weight));

  name_ = "body6d_" + robot.name() + "_" + bodyName;
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
  MetaTask::removeFromSolver(*positionTask, solver);
  MetaTask::removeFromSolver(*orientationTask, solver);
}

void EndEffectorTask::addToSolver(mc_solver::QPSolver & solver)
{
  MetaTask::addToSolver(*positionTask, solver);
  MetaTask::addToSolver(*orientationTask, solver);
}

void EndEffectorTask::update()
{
  MetaTask::update(*positionTask);
  MetaTask::update(*orientationTask);
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

void EndEffectorTask::dimWeight(const Eigen::VectorXd & dimW)
{
  assert(dimW.size() == 6);
  orientationTask->dimWeight(dimW.head(3));
  positionTask->dimWeight(dimW.tail(3));
}

Eigen::VectorXd EndEffectorTask::dimWeight() const
{
  Eigen::VectorXd ret(6);
  ret << orientationTask->dimWeight(), positionTask->dimWeight();
  return ret;
}

void EndEffectorTask::selectActiveJoints(mc_solver::QPSolver & solver,
                                const std::vector<std::string> & activeJointsName)
{
  positionTask->selectActiveJoints(solver, activeJointsName);
  orientationTask->selectActiveJoints(solver, activeJointsName);
}

void EndEffectorTask::selectUnactiveJoints(mc_solver::QPSolver & solver,
                                  const std::vector<std::string> & unactiveJointsName)
{
  positionTask->selectUnactiveJoints(solver, unactiveJointsName);
  orientationTask->selectUnactiveJoints(solver, unactiveJointsName);
}

void EndEffectorTask::resetJointsSelector(mc_solver::QPSolver & solver)
{
  positionTask->resetJointsSelector(solver);
  orientationTask->resetJointsSelector(solver);
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

void EndEffectorTask::addToLogger(mc_rtc::Logger & logger)
{
  logger.addLogEntry(name_ + "_target",
                     [this]() -> const sva::PTransformd &
                     {
                     return curTransform;
                     });
  logger.addLogEntry(name_,
                     [this]() -> const sva::PTransformd&
                     {
                     return robots.robot(robotIndex).mbc().bodyPosW[bodyIndex];
                     });
}

void EndEffectorTask::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntry(name_ + "_target");
  logger.removeLogEntry(name_);
}

}
