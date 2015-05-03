#include <mc_tasks/MoveContactTask.h>

namespace mc_tasks
{

MoveContactTask::MoveContactTask(mc_rbdyn::Robots & robots, mc_rbdyn::Contact & contact,
                                 mc_rbdyn::StanceConfig & config, double positionWStartPercent)
: robots(robots), robot(robots.robot()), env(robots.env()),
  robotSurf(contact.robotSurface), envSurf(contact.envSurface),
  robotBodyIndex(robot.bodyIndexByName(robotSurf->bodyName)),
  robotBodyId(robot.bodyIdByName(robotSurf->bodyName)),
  envBodyIndex(env.bodyIndexByName(envSurf->bodyName)),
  envBodyId(env.bodyIdByName(envSurf->bodyName)),
  targetTf(contact.X_0_rs(env)), targetPos(targetTf.translation()),
  targetOri(robotSurf->X_b_s().rotation().transpose()*targetTf.rotation()),
  normal(targetTf.rotation().row(2)),
  preTargetPos(targetPos + normal*config.contactObj.preContactDist),
  wp(config.contactTask.waypointConf.pos(robotSurfacePos(), targetTf, normal)),
  posStiff(config.contactTask.position.stiffness),
  extraPosStiff(config.contactTask.position.extraStiffness),
  positionTask(new tasks::qp::PositionTask(robots.mbs, 0, robotBodyId, robotSurfacePos().translation(), robotSurf->X_b_s().translation())),
  positionTaskSp(new tasks::qp::SetPointTask(robots.mbs, 0, positionTask.get(), posStiff, config.contactTask.position.weight*positionWStartPercent)),
  positionTaskSm(
    std::bind(static_cast<void (tasks::qp::SetPointTask::*)(double)>(&tasks::qp::SetPointTask::weight), positionTaskSp.get(), std::placeholders::_1),
    std::bind(static_cast<double (tasks::qp::SetPointTask::*)() const>(&tasks::qp::SetPointTask::weight), positionTaskSp.get()),
    std::bind(static_cast<void (tasks::qp::PositionTask::*)(const Eigen::Vector3d&)>(&tasks::qp::PositionTask::position), positionTask.get(), std::placeholders::_1),
    std::bind(static_cast<const Eigen::Vector3d & (tasks::qp::PositionTask::*)() const>(&tasks::qp::PositionTask::position), positionTask.get()),
    config.contactTask.position.weight, robotSurfacePos().translation(), 1),
  orientationTask(new tasks::qp::OrientationTask(robots.mbs, 0, robotBodyId, robot.mbc->bodyPosW[robotBodyIndex].rotation())),
  orientationTaskSp(new tasks::qp::SetPointTask(robots.mbs, 0, orientationTask.get(), config.contactTask.orientation.stiffness, config.contactTask.orientation.weight))
{
}

void MoveContactTask::toWaypoint(mc_rbdyn::StanceConfig & config, double positionSmoothPercent)
{
  target(wp, targetOri, config, positionSmoothPercent);
}

void MoveContactTask::toPreEnv(mc_rbdyn::StanceConfig & config, double positionSmoothPercent)
{
  target(preTargetPos, targetOri, config, positionSmoothPercent);
}

void MoveContactTask::toEnv(mc_rbdyn::StanceConfig & config, double positionSmoothPercent)
{
  target(targetPos, targetOri, config, positionSmoothPercent);
}

void MoveContactTask::target(const Eigen::Vector3d & pos, const Eigen::Matrix3d & ori, mc_rbdyn::StanceConfig & config, double positionSmoothPercent)
{
  const auto & positionConf = config.contactTask.position;
  const auto & orientationConf = config.contactTask.orientation;

  posStiff = positionConf.stiffness;
  extraPosStiff = positionConf.extraStiffness;
  positionTaskSp->stiffness(posStiff);
  positionTaskSm.reset(positionConf.weight, pos, positionSmoothPercent);

  orientationTaskSp->weight(orientationConf.weight);
  orientationTaskSp->stiffness(orientationConf.stiffness);
  orientationTask->orientation(ori);
}

sva::PTransformd MoveContactTask::robotSurfacePos()
{
  return robotSurf->X_0_s(robot);
}

sva::MotionVecd MoveContactTask::robotSurfaceVel()
{
  Eigen::Vector3d T_b_s = robotSurf->X_b_s().translation();
  Eigen::Matrix3d E_0_b = robot.mbc->bodyPosW[robotBodyIndex].rotation();
  sva::PTransformd pts(E_0_b.transpose(), T_b_s);
  return pts*robot.mbc->bodyVelB[robotBodyIndex];
}

void MoveContactTask::addToSolver(tasks::qp::QPSolver & solver)
{
  solver.addTask(positionTaskSp.get());
  solver.addTask(orientationTaskSp.get());
  solver.updateTasksNrVars(robots.mbs);
}

void MoveContactTask::removeFromSolver(tasks::qp::QPSolver & solver)
{
  solver.removeTask(orientationTaskSp.get());
  solver.removeTask(positionTaskSp.get());
}

void MoveContactTask::update()
{
  positionTaskSm.update();
  double err = (robotSurfacePos().translation() - preTargetPos).norm();
  double extra = extraStiffness(err, extraPosStiff);
  positionTaskSp->stiffness(posStiff + extra);
}

}
