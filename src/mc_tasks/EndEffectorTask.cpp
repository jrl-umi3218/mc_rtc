#include <mc_tasks/EndEffectorTask.h>

namespace mc_tasks
{

EndEffectorTask::EndEffectorTask(const std::string & bodyName, const mc_rbdyn::Robots & robots, unsigned int robotIndex)
: robots(robots), bodyName(bodyName), inSolver(false)
{
  const mc_rbdyn::Robot & robot = robots.robots[robotIndex];
  unsigned int bodyId = robot.bodyIdByName(bodyName);
  unsigned int bodyIndex = robot.bodyIndexByName(bodyName);
  sva::PTransformd bpw = robot.mbc->bodyPosW[bodyIndex];
  std::cout << "Creating task for body " << bodyName << std::endl;
  std::cout << "Current pos " << std::endl << bpw.translation() << std::endl;
  std::cout << "Current ori " << std::endl << bpw.rotation() << std::endl;

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

void EndEffectorTask::removeFromSolver(tasks::qp::QPSolver & solver)
{
  if(inSolver)
  {
    solver.removeTask(positionTaskSp.get());
    solver.removeTask(orientationTaskSp.get());
    solver.updateConstrsNrVars(robots.mbs);
    solver.updateConstrSize();
    inSolver = false;
  }
}

void EndEffectorTask::addToSolver(tasks::qp::QPSolver & solver)
{
  if(!inSolver)
  {
    solver.addTask(positionTaskSp.get());
    solver.addTask(orientationTaskSp.get());
    solver.updateTasksNrVars(robots.mbs);
    solver.updateConstrsNrVars(robots.mbs);
    solver.updateConstrSize();
    inSolver = true;
  }
}

void EndEffectorTask::add_ef_pose(const sva::PTransformd & dtr)
{
  Eigen::Matrix3d new_rot = curTransform.rotation()*dtr.rotation();
  Eigen::Vector3d new_t = curTransform.translation() + dtr.translation();
  curTransform = sva::PTransformd(new_rot, new_t);
  positionTask->position(curTransform.translation());
  orientationTask->orientation(curTransform.rotation());
}

void EndEffectorTask::set_ef_pose(const sva::PTransformd & tf)
{
  positionTask->position(tf.translation());
  orientationTask->orientation(tf.rotation());
}

sva::PTransformd EndEffectorTask::get_ef_pose()
{
  return sva::PTransformd(orientationTask->orientation(), positionTask->position());
}

}
