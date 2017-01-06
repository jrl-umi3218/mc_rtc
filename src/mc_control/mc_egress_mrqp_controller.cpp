#include "mc_egress_mrqp_controller.h"

#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Surface.h>

#include <mc_rtc/logging.h>

#include <Tasks/QPContactConstr.h>

#include "mc_mr_egress_phases.cpp"

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <array>

namespace mc_control
{

MCEgressMRQPController::MCEgressMRQPController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt)
  : MCController({robot_module, mc_rbdyn::RobotLoader::get_robot_module("PolarisRangerEgress"), mc_rbdyn::RobotLoader::get_robot_module("env", std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH), std::string("ground"))}, dt),
    egressContacts(),
    collsConstraint(robots(), 0, 1, timeStep),
    curPhase(START),
    execPhase(new EgressMRStartPhase)
{
  std::array<double, 3> damper = {{0.1, 0.01, 0.01}};
  polarisKinematicsConstraint = mc_solver::KinematicsConstraint(robots(), 1, timeStep, damper, 0.5);
  collsConstraint.addCollisions(solver(),
      {
        mc_rbdyn::Collision("RLEG_LINK5", "left_column", 0.1, 0.05, 0.),
        mc_rbdyn::Collision("RLEG_LINK4", "left_column", 0.1, 0.05, 0.),
        mc_rbdyn::Collision("RARM_LINK6", "left_column", 0.1, 0.05, 0.),
        mc_rbdyn::Collision("RARM_LINK5", "left_column", 0.1, 0.05, 0.),
        mc_rbdyn::Collision("RARM_LINK4", "left_column", 0.1, 0.05, 0.)
        });
  selfCollisionConstraint.addCollisions(solver(),
      {
      mc_rbdyn::Collision("LLEG_LINK3", "LARM_LINK3", 0.05, 0.01, 0.),
      mc_rbdyn::Collision("LLEG_LINK4", "LARM_LINK3", 0.05, 0.01, 0.)
      });
  qpsolver->addConstraintSet(contactConstraint);
  qpsolver->addConstraintSet(kinematicsConstraint);
  qpsolver->addConstraintSet(polarisKinematicsConstraint);
  qpsolver->addConstraintSet(selfCollisionConstraint);
  qpsolver->addConstraintSet(collsConstraint);

  qpsolver->addTask(postureTask.get());
  postureTask->weight(1);

  mc_rbdyn::Robot& polaris = robots().robot(1);

  robot().mbc().q[0] = {1, 0, 0, 0, 0, 0, 0.76};

  rbd::forwardKinematics(robot().mb(), robot().mbc());
  rbd::forwardVelocity(robot().mb(), robot().mbc());

  egressContacts.emplace_back(robots(), robots().robotIndex(), 1,
                           "Butthock", "lazy_seat");
  egressContacts.emplace_back(robots(), robots().robotIndex(), 1,
                           "LFullSole", "exit_platform");
  egressContacts.emplace_back(robots(), robots().robotIndex(), 1,
                           "RightGripper", "bar_wheel");
  qpsolver->setContacts(egressContacts);

  polarisPostureTask.reset(new tasks::qp::PostureTask(robots().mbs(), 1, robots().robot(1).mbc().q, 1.0, 1));
  lazyPostureTask.reset(new tasks::qp::PostureTask(robots().mbs(), 1, polaris.mbc().q, 0.0, 1000.0));
  std::vector<tasks::qp::JointStiffness> jsv;
  jsv.push_back({"lazy_susan", 0.1});
  lazyPostureTask->jointsStiffness(robots().mbs(), jsv);

  comTask.reset(new mc_tasks::CoMTask(robots(), robots().robotIndex()));
  comTask->stiffness(1.);
  efTask.reset(new mc_tasks::EndEffectorTask("RARM_LINK6", robots(),
                                             robots().robotIndex()));
  torsoOriTask.reset(new mc_tasks::OrientationTask("CHEST_LINK1", robots(),
                                                   robots().robotIndex(),
                                                   1., 1.));

  LOG_SUCCESS("MCEgressMRQPController init done")
}

bool MCEgressMRQPController::run()
{
  bool success = MCController::run();
  if(success)
  {
    bool next = execPhase->run(*this);
    if(next)
    {
      play_next_stance();
    }
  }
  else
  {
    LOG_ERROR("Failed to run")
  }
  return success;
}

void MCEgressMRQPController::reset(const ControllerResetData & reset_data)
{
  MCController::reset(reset_data);
  LOG_INFO("Enter mr egress reset")
  robot().mbc().zero(robot().mb());
  robot().mbc().q = reset_data.q;
  robot().mbc().q[0] = {1, 0, 0, 0, 0, 0, 0.76};
  rbd::forwardKinematics(robot().mb(), robot().mbc());
  rbd::forwardVelocity(robot().mb(), robot().mbc());
  postureTask->posture(robot().mbc().q);

  resetBasePose();
  resetWheelTransform();
  resetLazyTransform();

  qpsolver->addTask(polarisPostureTask.get());
  qpsolver->addTask(lazyPostureTask.get());

  qpsolver->setContacts(egressContacts);

  LOG_INFO("End mr egress reset")
}

void MCEgressMRQPController::addCollision(const mc_rbdyn::Collision& coll)
{
  collsConstraint.addCollisions(solver(), {coll});
}


void MCEgressMRQPController::resetWheelTransform()
{
  sva::PTransformd graspOffset(sva::RotX(-M_PI/2), Eigen::Vector3d(0, 0, 0));
  mc_rbdyn::Robot& polaris = robots().robot(1);
  //Change wheel position
  unsigned int chassis_index = polaris.bodyIndexByName("chassis");
  //Do not take into account potential rotation of steering wheel
  unsigned int joint_index = polaris.jointIndexByName("adjust_steering_wheel");

  const auto & gripperSurface = robot().surface("RightGripper");
  const auto & wheelSurface = polaris.surface("bar_wheel");

  sva::PTransformd X_wheel_s = graspOffset*wheelSurface.X_b_s();

  sva::PTransformd X_0_s = gripperSurface.X_0_s(robot(), robot().mbc());

  sva::PTransformd X_0_chassis = polaris.mbc().bodyPosW[chassis_index];

  sva::PTransformd X_chassis_wheel = (X_wheel_s).inv()*X_0_s*(X_0_chassis).inv();

  polaris.mb().transform(static_cast<int>(joint_index), X_chassis_wheel);
  polaris.mbc().zero(polaris.mb());

  rbd::forwardKinematics(polaris.mb(), polaris.mbc());
  rbd::forwardVelocity(polaris.mb(), polaris.mbc());
}

void MCEgressMRQPController::resetLazyTransform()
{
  mc_rbdyn::Robot& polaris = robots().robot(1);
  //Change wheel position
  unsigned int chassis_index = polaris.bodyIndexByName("chassis");
  //Do not take unsigned into account potential rotation of steering wheel
  unsigned int joint_index = polaris.jointIndexByName("lazy_susan");

  const auto & gripperSurface = robot().surface("Butthock");
  const auto & wheelSurface = polaris.surface("lazy_seat");

  sva::PTransformd X_wheel_s = wheelSurface.X_b_s();

  sva::PTransformd X_0_s = gripperSurface.X_0_s(robot(), robot().mbc());

  sva::PTransformd X_0_chassis = polaris.mbc().bodyPosW[chassis_index];

  sva::PTransformd X_chassis_wheel = (X_wheel_s).inv()*X_0_s*(X_0_chassis).inv();

  polaris.mb().transform(static_cast<int>(joint_index), X_chassis_wheel);
  polaris.mbc().zero(polaris.mb());

  rbd::forwardKinematics(polaris.mb(), polaris.mbc());
  rbd::forwardVelocity(polaris.mb(), polaris.mbc());
}

void MCEgressMRQPController::resetBasePose()
{
  mc_rbdyn::Robot& polaris = robots().robot(1);
  //Reset freeflyer, compute its position frow wheel and re-set it
  robot().mbc().q[0] = {1., 0., 0., 0., 0., 0., 0.};
  rbd::forwardKinematics(robot().mb(), robot().mbc());
  rbd::forwardVelocity(robot().mb(), robot().mbc());

  auto X_0_w = polaris.surface("exit_platform").X_0_s(polaris);
  sva::PTransformd graspOffset(sva::RotZ(M_PI/4), Eigen::Vector3d(-0.1, -0.1, 0));
  auto X_0_s = robot().surface("LFullSole").X_0_s(robot());
  auto X_0_base = X_0_s.inv()*(graspOffset*X_0_w);

  const auto quat = Eigen::Quaterniond(X_0_base.rotation()).inverse();
  const Eigen::Vector3d trans(X_0_base.translation());
  std::vector<double> baseQ = {quat.w(), quat.x(), quat.y(), quat.z(),
                               trans.x(), trans.y(), trans.z()};

  robot().mbc().q[0] = baseQ;
  rbd::forwardKinematics(robot().mb(), robot().mbc());
  rbd::forwardVelocity(robot().mb(), robot().mbc());
}

bool MCEgressMRQPController::play_next_stance()
{
  switch(curPhase)
  {
  case START:
    curPhase = ROTATELAZY;
    execPhase.reset(new EgressRotateLazyPhase);
    break;
  case ROTATELAZY:
  //  curPhase = REPLACELEFTFOOT;
  //  execPhase.reset(new EgressReplaceLeftFootPhase);
  //  break;
  //case REPLACELEFTFOOT:
    curPhase = STANDUP;
    execPhase.reset(new EgressMRStandupPhase(Eigen::Vector3d(0.0, 0.05, -0.25)));
    break;
  //  curPhase = PLACERIGHTFOOT;
  //  execPhase.reset(new EgressPlaceRightFootPhase);
  //  break;
  //case PLACERIGHTFOOT:
  //  curPhase = STANDUP;
  //  execPhase.reset(new EgressMRStandupPhase(-0.15));
  //  break;
  case STANDUP:
    curPhase = MOVECOMLEFT;
    execPhase.reset(new EgressMoveComSurfPhase("LFullSole", 0.15));
    break;
  case MOVECOMLEFT:
    curPhase = REPLACERIGHTFOOT;
    execPhase.reset(new EgressReplaceRightFootPhase);
    break;
  case REPLACERIGHTFOOT:
    curPhase = MOVECOMRIGHT;
    execPhase.reset(new EgressMoveComSurfPhase("RFullSole", 0.15, true));
    break;
  case MOVECOMRIGHT:
    curPhase = REPLACELEFTFOOT;
    execPhase.reset(new EgressReplaceLeftFootPhase);
    break;
  case REPLACELEFTFOOT:
    curPhase = MOVECOMFORCELEFT;
    execPhase.reset(new EgressMoveComSurfPhase("LFullSole", 0.10, true));
    //Use this to lift the rear feet by a maximum of 10cm (last arg)
    //execPhase.reset(new EgressMoveComForcePhase("LFullSole", 0.10, 0.1));
    break;
  case MOVECOMFORCELEFT:
    curPhase = PUTDOWNRIGHTFOOT;
    execPhase.reset(new EgressPutDownRightFootPhase);
    break;
  case PUTDOWNRIGHTFOOT:
    curPhase = CENTERCOM;
    comTask->weight(1000.);
    execPhase.reset(new EgressCenterComPhase(0.10));
    break;
  case CENTERCOM:
    solver().removeTask(torsoOriTask);
    postureTask->weight(1.);
    curPhase = OPENGRIPPER;
    execPhase.reset(new EgressOpenRightGripperPhase);
    break;
  case OPENGRIPPER:
    curPhase = REMOVEHAND;
    execPhase.reset(new EgressRemoveRightGripperPhase(10, 0.02, -10));
    break;
  case REMOVEHAND:
    curPhase = REPLACEHAND;
    execPhase.reset(new EgressReplaceRightHandPhase);
    break;
  default:
    LOG_SUCCESS("Done")
    break;
  }
  return true;
}

std::vector<std::string> MCEgressMRQPController::supported_robots() const
{
  return {"hrp2_drc"};
}

}
