#include <mc_tasks/MoveContactTask.h>

#include <mc_rbdyn/Contact.h>
#include <mc_rbdyn/Surface.h>

#include <mc_rbdyn/rpy_utils.h>

namespace mc_tasks
{

MoveContactTask::MoveContactTask(mc_rbdyn::Robots & robots, mc_rbdyn::Robot & robot, mc_rbdyn::Robot & env, mc_rbdyn::Contact & contact,
                                 mc_rbdyn::StanceConfig & config, double positionWStartPercent)
: MoveContactTask(robots, robot, env, contact,
                  config.contactTask.position.stiffness,
                  config.contactTask.position.extraStiffness,
                  config.contactTask.position.weight,
                  config.contactTask.orientation.stiffness,
                  config.contactTask.orientation.weight,
                  config.contactObj.preContactDist,
                  config.contactTask.waypointConf.pos,
                  config.contactObj.adjustOffset,
                  config.contactObj.adjustRPYOffset,
                  positionWStartPercent)
{
}

MoveContactTask::MoveContactTask(mc_rbdyn::Robots & robots, mc_rbdyn::Robot & robot,
                  mc_rbdyn::Robot & env, mc_rbdyn::Contact & contact,
                  double posStiffness, double extraPosStiffness, double posWeight,
                  double oriStiffness, double oriWeight,
                  double preContactDist,
                  mc_rbdyn::WaypointFunction waypointPos,
                  const Eigen::Vector3d & adjustOffset,
                  const Eigen::Vector3d & adjustRPYOffset,
                  double positionWStartPercent)
: robots(robots), robot(robot), env(env),
  robotSurf(contact.r1Surface()), envSurf(contact.r2Surface())
{
  robotBodyIndex = robot.bodyIndexByName(robotSurf->bodyName());
  envBodyIndex = env.bodyIndexByName(envSurf->bodyName());

  set_target_tf(contact.X_0_r1s(robots),
                preContactDist,
                waypointPos,
                adjustOffset,
                adjustRPYOffset);

  posStiff = posStiffness;
  extraPosStiff = extraPosStiffness;

  positionTask.reset(new tasks::qp::PositionTask(robots.mbs(), 0, robotSurf->bodyName(), robotSurfacePos().translation(), robotSurf->X_b_s().translation()));
  positionTaskSp.reset(new tasks::qp::SetPointTask(robots.mbs(), 0, positionTask.get(), posStiff, posWeight*positionWStartPercent));
  positionTaskSm.reset(new SmoothTask<Eigen::Vector3d>(
    std::bind(static_cast<void (tasks::qp::SetPointTask::*)(double)>(&tasks::qp::SetPointTask::weight), positionTaskSp.get(), std::placeholders::_1),
    std::bind(static_cast<double (tasks::qp::SetPointTask::*)() const>(&tasks::qp::SetPointTask::weight), positionTaskSp.get()),
    std::bind(static_cast<void (tasks::qp::PositionTask::*)(const Eigen::Vector3d&)>(&tasks::qp::PositionTask::position), positionTask.get(), std::placeholders::_1),
    std::bind(static_cast<const Eigen::Vector3d & (tasks::qp::PositionTask::*)() const>(&tasks::qp::PositionTask::position), positionTask.get()),
    posWeight, robotSurfacePos().translation(), 1));
  orientationTask.reset(new tasks::qp::OrientationTask(robots.mbs(), 0, robotSurf->bodyName(), robot.mbc().bodyPosW[robotBodyIndex].rotation()));
  orientationTaskSp.reset(new tasks::qp::SetPointTask(robots.mbs(), 0, orientationTask.get(), oriStiffness, oriWeight));
  useSmoothTask = true;
}

void MoveContactTask::toWaypoint(mc_rbdyn::StanceConfig & config, double positionSmoothPercent)
{
  target(wp, targetOri,
         config.contactTask.position.stiffness,
         config.contactTask.position.extraStiffness,
         config.contactTask.position.weight,
         config.contactTask.orientation.stiffness,
         config.contactTask.orientation.weight,
         positionSmoothPercent);
}

void MoveContactTask::toWaypoint(double posStiffness,
                                 double extraPosStiffness, double posWeight,
                                 double oriStiffness, double oriWeight,
                                 double positionSmoothPercent)
{
  target(wp, targetOri,
         posStiffness, extraPosStiffness, posWeight,
         oriStiffness, oriWeight,
         positionSmoothPercent);
}

void MoveContactTask::toWaypoint()
{
  target(wp, targetOri,
         posStiff, extraPosStiff, positionTaskSm->weight,
         orientationTaskSp->stiffness(), orientationTaskSp->weight(), 1.0);
}

void MoveContactTask::toPreEnv(mc_rbdyn::StanceConfig & config, double positionSmoothPercent)
{
  target(preTargetPos, targetOri,
         config.contactTask.position.stiffness,
         config.contactTask.position.extraStiffness,
         config.contactTask.position.weight,
         config.contactTask.orientation.stiffness,
         config.contactTask.orientation.weight,
         positionSmoothPercent);
}

void MoveContactTask::toPreEnv(double posStiffness,
                                 double extraPosStiffness, double posWeight,
                                 double oriStiffness, double oriWeight,
                                 double positionSmoothPercent)
{
  target(preTargetPos, targetOri,
         posStiffness, extraPosStiffness, posWeight,
         oriStiffness, oriWeight,
         positionSmoothPercent);
}

void MoveContactTask::toPreEnv()
{
  target(preTargetPos, targetOri,
         posStiff, extraPosStiff, positionTaskSm->weight,
         orientationTaskSp->stiffness(), orientationTaskSp->weight(), 1.0);
}


void MoveContactTask::target(const Eigen::Vector3d & pos, const Eigen::Matrix3d & ori,
                             double posStiffness, double extraPosStiffness, double posWeight,
                             double oriStiffness, double oriWeight,
                             double positionSmoothPercent)
{
  posStiff = posStiffness;
  extraPosStiff = extraPosStiffness;
  positionTaskSp->stiffness(posStiff);
  positionTaskSm->reset(posWeight, pos, positionSmoothPercent);

  orientationTaskSp->weight(oriWeight);
  orientationTaskSp->stiffness(oriStiffness);
  orientationTask->orientation(ori);
}

sva::PTransformd MoveContactTask::robotSurfacePos()
{
  return robotSurf->X_0_s(robot);
}

sva::MotionVecd MoveContactTask::robotSurfaceVel()
{
  Eigen::Vector3d T_b_s = robotSurf->X_b_s().translation();
  Eigen::Matrix3d E_0_b = robot.mbc().bodyPosW[robotBodyIndex].rotation();
  sva::PTransformd pts(E_0_b.transpose(), T_b_s);
  return pts*robot.mbc().bodyVelB[robotBodyIndex];
}

void MoveContactTask::addToSolver(mc_solver::QPSolver & solver)
{
  solver.addTask(positionTaskSp.get());
  solver.addTask(orientationTaskSp.get());
}

void MoveContactTask::removeFromSolver(mc_solver::QPSolver & solver)
{
  solver.removeTask(orientationTaskSp.get());
  solver.removeTask(positionTaskSp.get());
}

void MoveContactTask::update()
{
  if(useSmoothTask)
  {
    positionTaskSm->update();
  }
  double err = (robotSurfacePos().translation() - preTargetPos).norm();
  double extra = extraStiffness(err, extraPosStiff);
  positionTaskSp->stiffness(posStiff + extra);
}

void MoveContactTask::set_target_tf(const sva::PTransformd & X_target, mc_rbdyn::StanceConfig & config)
{
  set_target_tf(X_target,
                config.contactObj.preContactDist,
                config.contactTask.waypointConf.pos,
                config.contactObj.adjustOffset,
                config.contactObj.adjustRPYOffset);
}


void MoveContactTask::set_target_tf(const sva::PTransformd & X_target,
                                    double preContactDist,
                                    mc_rbdyn::WaypointFunction waypointPos,
                     const Eigen::Vector3d & adjustOffset,
                     const Eigen::Vector3d & adjustRPYOffset)
{
  targetTf = X_target;
  targetPos = targetTf.translation() + adjustOffset;
  targetOri = mc_rbdyn::rpyToMat(adjustRPYOffset)*robotSurf->X_b_s().rotation().transpose()*targetTf.rotation();
  normal = targetTf.rotation().row(2);
  preTargetPos = targetPos + normal*preContactDist;
  wp = waypointPos(robotSurfacePos(), targetTf, normal);
}

Eigen::VectorXd MoveContactTask::eval() const
{
  Eigen::Vector6d err;
  err << orientationTask->eval(), positionTask->eval();
  return err;
}

Eigen::VectorXd MoveContactTask::speed() const
{
  Eigen::Vector6d spd;
  spd << orientationTask->speed(), positionTask->speed();
  return spd;
}

}
