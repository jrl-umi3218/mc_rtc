#include <mc_tasks/EndEffectorTask.h>

namespace mc_tasks
{

EndEffectorTask::EndEffectorTask(const std::string & bodyName, const mc_rbdyn::Robots & robots, unsigned int robotIndex, double stiffness, double weight)
: robots(robots), robotIndex(robotIndex),
  bodyName(bodyName), inSolver(false)
{
  const mc_rbdyn::Robot & robot = robots.robot(robotIndex);
  int bodyId = robot.bodyIdByName(bodyName);
  unsigned int bodyIndex = robot.bodyIndexByName(bodyName);
  sva::PTransformd bpw = robot.mbc().bodyPosW[bodyIndex];

  curTransform = bpw;

  positionTask.reset(new tasks::qp::PositionTask(robots.mbs(), static_cast<int>(robotIndex), bodyId, bpw.translation(), Eigen::Vector3d(0,0,0)));
  positionTaskSp.reset(new tasks::qp::SetPointTask(robots.mbs(), static_cast<int>(robotIndex), positionTask.get(), stiffness, weight));

  orientationTask.reset(new tasks::qp::OrientationTask(robots.mbs(), static_cast<int>(robotIndex), bodyId, bpw.rotation()));
  orientationTaskSp.reset(new tasks::qp::SetPointTask(robots.mbs(), static_cast<int>(robotIndex), orientationTask.get(), stiffness, weight));

  err = Eigen::VectorXd(dim());
  spd = Eigen::VectorXd(dim());
}

void EndEffectorTask::resetTask(const mc_rbdyn::Robots & robots, unsigned int robotIndex)
{
  const mc_rbdyn::Robot & robot = robots.robot(robotIndex);
  unsigned int bodyIndex = robot.bodyIndexByName(bodyName);

  curTransform = robot.mbc().bodyPosW[bodyIndex];
  positionTask->position(curTransform.translation());
  orientationTask->orientation(curTransform.rotation());
}

void EndEffectorTask::removeFromSolver(mc_solver::QPSolver & solver)
{
  if(inSolver)
  {
    solver.removeTask(positionTaskSp.get());
    solver.removeTask(orientationTaskSp.get());
    inSolver = false;
  }
}

void EndEffectorTask::addToSolver(mc_solver::QPSolver & solver)
{
  if(!inSolver)
  {
    solver.addTask(positionTaskSp.get());
    solver.addTask(orientationTaskSp.get());
    inSolver = true;
  }
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

int EndEffectorTask::dim()
{
  return positionTask->dim()+orientationTask->dim();
}

const Eigen::VectorXd& EndEffectorTask::eval()
{
  err << orientationTask->eval(), positionTask->eval();
  return err;
}

const Eigen::VectorXd& EndEffectorTask::speed()
{
  spd << orientationTask->speed(), positionTask->speed();
  return spd;
}

}
