#include <mc_tasks/EndEffectorTask.h>

namespace mc_tasks
{

EndEffectorTask::EndEffectorTask(const std::string & bodyName, const mc_rbdyn::Robots & robots, unsigned int robotIndex)
: bodyName(bodyName)
{
  const mc_rbdyn::Robot & robot = robots.robots[robotIndex];
  unsigned int bodyId = robot.bodyIdByName(bodyName);
  unsigned int bodyIndex = robot.bodyIndexByName(bodyName);
  sva::PTransformd bpw = robot.mbc->bodyPosW[bodyIndex];

  curTransform = bpw;

  positionTask.reset(new tasks::qp::PositionTask(robots.mbs, robotIndex, bodyId, bpw.translation(), Eigen::Vector3d(0,0,0)));
  positionTaskSp.reset(new tasks::qp::SetPointTask(robots.mbs, robotIndex, positionTask.get(), 2, 1000));

  orientationTask.reset(new tasks::qp::OrientationTask(robots.mbs, robotIndex, bodyId, bpw.rotation()));
  orientationTaskSp.reset(new tasks::qp::SetPointTask(robots.mbs, robotIndex, orientationTask.get(), 2, 1000));
}

void EndEffectorTask::resetTask(const mc_rbdyn::Robots & robots, unsigned int robotIndex)
{
  const mc_rbdyn::Robot & robot = robots.robots[robotIndex];
  unsigned int bodyIndex = robot.bodyIndexByName(bodyName);

  curTransform = robot.mbc->bodyPosW[bodyIndex];
  positionTask->position(curTransform.translation());
  orientationTask->orientation(curTransform.rotation());
}

void EndEffectorTask::removeFromSolver(mc_solver::QPSolver & qpsolver)
{
  qpsolver.solver.removeTask(positionTaskSp.get());
  qpsolver.solver.removeTask(orientationTaskSp.get());
  qpsolver.update();
}

void EndEffectorTask::addToSolver(mc_solver::QPSolver & qpsolver)
{
  qpsolver.solver.addTask(positionTaskSp.get());
  qpsolver.solver.addTask(orientationTaskSp.get());
  qpsolver.update();
}

void EndEffectorTask::add_ef_pose(const sva::PTransformd & dtr)
{
  Eigen::Matrix3d new_rot = curTransform.rotation()*dtr.rotation();
  Eigen::Vector3d new_t = curTransform.translation() + dtr.translation();
  curTransform = sva::PTransformd(new_rot, new_t);
  positionTask->position(curTransform.translation());
  orientationTask->orientation(curTransform.rotation());
}

}
