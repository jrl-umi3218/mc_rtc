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
  name_ = "move_contact_" +
            robot.name() + "_" + contact.r1Surface()->name() + "_" +
            env.name() + "_" + contact.r2Surface()->name();
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
  if(!inSolver)
  {
    inSolver = true;
    solver.addTask(positionTaskSp.get());
    solver.addTask(orientationTaskSp.get());
  }
}

void MoveContactTask::removeFromSolver(mc_solver::QPSolver & solver)
{
  if(inSolver)
  {
    inSolver = false;
    solver.removeTask(orientationTaskSp.get());
    solver.removeTask(positionTaskSp.get());
  }
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

void MoveContactTask::dimWeight(const Eigen::VectorXd & dimW)
{
  assert(dimW.size() == 6);
  positionTaskSp->dimWeight(dimW.head(3));
  orientationTaskSp->dimWeight(dimW.tail(3));
}

Eigen::VectorXd MoveContactTask::dimWeight() const
{
  Eigen::Vector6d ret;
  ret << positionTaskSp->dimWeight(), orientationTaskSp->dimWeight();
  return ret;
}

void MoveContactTask::selectActiveJoints(mc_solver::QPSolver & solver,
                                const std::vector<std::string> & aJN)
{
  bool putBack = inSolver;
  if(putBack)
  {
    removeFromSolver(solver);
  }

  /* JS on position task */
  double positionStiff = positionTaskSp->stiffness();
  double positionW = positionTaskSm->weight;
  double positionWPercent = 1/static_cast<double>(positionTaskSm->nrIter);
  positionJSTask = std::make_shared<tasks::qp::JointsSelector>(tasks::qp::JointsSelector::ActiveJoints(robots.mbs(), 0, positionTask.get(), aJN));
  positionTaskSp = std::make_shared<tasks::qp::SetPointTask>(robots.mbs(), 0, positionJSTask.get(), positionStiff, positionW*positionWPercent);
  positionTaskSm.reset(new SmoothTask<Eigen::Vector3d>(
    std::bind(static_cast<void (tasks::qp::SetPointTask::*)(double)>(&tasks::qp::SetPointTask::weight), positionTaskSp.get(), std::placeholders::_1),
    std::bind(static_cast<double (tasks::qp::SetPointTask::*)() const>(&tasks::qp::SetPointTask::weight), positionTaskSp.get()),
    std::bind(static_cast<void (tasks::qp::PositionTask::*)(const Eigen::Vector3d&)>(&tasks::qp::PositionTask::position), positionTask.get(), std::placeholders::_1),
    std::bind(static_cast<const Eigen::Vector3d & (tasks::qp::PositionTask::*)() const>(&tasks::qp::PositionTask::position), positionTask.get()),
    positionW, robotSurfacePos().translation(), 1));

  /* JS on orientation task */
  orientationJSTask = std::make_shared<tasks::qp::JointsSelector>(tasks::qp::JointsSelector::ActiveJoints(robots.mbs(), 0, orientationTask.get(), aJN));
  double oriStiffness = orientationTaskSp->stiffness();
  double oriWeight = orientationTaskSp->weight();
  orientationTaskSp = std::make_shared<tasks::qp::SetPointTask>(robots.mbs(), 0, orientationJSTask.get(), oriStiffness, oriWeight);
  orientationTaskSp = std::make_shared<tasks::qp::SetPointTask>(robots.mbs(), 0, orientationJSTask.get(), oriStiffness, oriWeight);
  if(putBack)
  {
    addToSolver(solver);
  }
}

void MoveContactTask::selectUnactiveJoints(mc_solver::QPSolver & solver,
                                  const std::vector<std::string> & uJN)
{
  bool putBack = inSolver;
  if(putBack)
  {
    removeFromSolver(solver);
  }
  /* JS on position task */
  double positionStiff = positionTaskSp->stiffness();
  double positionW = positionTaskSm->weight;
  double positionWPercent = 1/static_cast<double>(positionTaskSm->nrIter);
  positionJSTask = std::make_shared<tasks::qp::JointsSelector>(tasks::qp::JointsSelector::UnactiveJoints(robots.mbs(), 0, positionTask.get(), uJN));
  positionTaskSp = std::make_shared<tasks::qp::SetPointTask>(robots.mbs(), 0, positionJSTask.get(), positionStiff, positionW*positionWPercent);
  positionTaskSm.reset(new SmoothTask<Eigen::Vector3d>(
    std::bind(static_cast<void (tasks::qp::SetPointTask::*)(double)>(&tasks::qp::SetPointTask::weight), positionTaskSp.get(), std::placeholders::_1),
    std::bind(static_cast<double (tasks::qp::SetPointTask::*)() const>(&tasks::qp::SetPointTask::weight), positionTaskSp.get()),
    std::bind(static_cast<void (tasks::qp::PositionTask::*)(const Eigen::Vector3d&)>(&tasks::qp::PositionTask::position), positionTask.get(), std::placeholders::_1),
    std::bind(static_cast<const Eigen::Vector3d & (tasks::qp::PositionTask::*)() const>(&tasks::qp::PositionTask::position), positionTask.get()),
    positionW, robotSurfacePos().translation(), 1));

  /* JS on orientation task */
  orientationJSTask = std::make_shared<tasks::qp::JointsSelector>(tasks::qp::JointsSelector::UnactiveJoints(robots.mbs(), 0, orientationTask.get(), uJN));
  double oriStiffness = orientationTaskSp->stiffness();
  double oriWeight = orientationTaskSp->weight();
  orientationTaskSp = std::make_shared<tasks::qp::SetPointTask>(robots.mbs(), 0, orientationJSTask.get(), oriStiffness, oriWeight);
  orientationTaskSp = std::make_shared<tasks::qp::SetPointTask>(robots.mbs(), 0, orientationJSTask.get(), oriStiffness, oriWeight);
  if(putBack)
  {
    addToSolver(solver);
  }
}

void MoveContactTask::resetJointsSelector(mc_solver::QPSolver & solver)
{
  bool putBack = inSolver;
  if(putBack)
  {
    removeFromSolver(solver);
  }
  /* JS on position task */
  double positionStiff = positionTaskSp->stiffness();
  double positionW = positionTaskSm->weight;
  double positionWPercent = 1/static_cast<double>(positionTaskSm->nrIter);
  positionJSTask = nullptr;
  positionTaskSp = std::make_shared<tasks::qp::SetPointTask>(robots.mbs(), 0, positionTask.get(), positionStiff, positionW*positionWPercent);
  positionTaskSm.reset(new SmoothTask<Eigen::Vector3d>(
    std::bind(static_cast<void (tasks::qp::SetPointTask::*)(double)>(&tasks::qp::SetPointTask::weight), positionTaskSp.get(), std::placeholders::_1),
    std::bind(static_cast<double (tasks::qp::SetPointTask::*)() const>(&tasks::qp::SetPointTask::weight), positionTaskSp.get()),
    std::bind(static_cast<void (tasks::qp::PositionTask::*)(const Eigen::Vector3d&)>(&tasks::qp::PositionTask::position), positionTask.get(), std::placeholders::_1),
    std::bind(static_cast<const Eigen::Vector3d & (tasks::qp::PositionTask::*)() const>(&tasks::qp::PositionTask::position), positionTask.get()),
    positionW, robotSurfacePos().translation(), 1));

  /* JS on orientation task */
  orientationJSTask = nullptr;
  double oriStiffness = orientationTaskSp->stiffness();
  double oriWeight = orientationTaskSp->weight();
  orientationTaskSp = std::make_shared<tasks::qp::SetPointTask>(robots.mbs(), 0, orientationTask.get(), oriStiffness, oriWeight);
  orientationTaskSp = std::make_shared<tasks::qp::SetPointTask>(robots.mbs(), 0, orientationTask.get(), oriStiffness, oriWeight);
  if(putBack)
  {
    addToSolver(solver);
  }
}

Eigen::VectorXd MoveContactTask::eval() const
{
  Eigen::Vector6d err;
  if(positionJSTask)
  {
    err << orientationJSTask->eval(), positionJSTask->eval();
  }
  else
  {
    err << orientationTask->eval(), positionTask->eval();
  }
  return err;
}

Eigen::VectorXd MoveContactTask::speed() const
{
  Eigen::Vector6d spd;
  if(positionJSTask)
  {
    spd << orientationJSTask->speed(), positionJSTask->speed();
  }
  else
  {
    spd << orientationTask->speed(), positionTask->speed();
  }
  return spd;
}

}
