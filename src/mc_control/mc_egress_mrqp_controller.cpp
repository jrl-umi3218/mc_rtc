#include <mc_control/mc_egress_mrqp_controller.h>
#include <mc_robots/polaris_ranger_egress.h>
#include <mc_rbdyn/robot.h>

#include <Tasks/QPContactConstr.h>

#include "mc_mr_egress_phases.cpp"

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

namespace mc_control
{

MCEgressMRQPController::MCEgressMRQPController(const std::vector<std::shared_ptr<mc_rbdyn::RobotModule> >& env_modules)
  : MCMRQPController(env_modules),
    polarisKinematicsConstraint(robots(), 1, timeStep, false,
        {0.1, 0.01, 0.01}, 0.5),
    egressContacts(),
    collsConstraint(robots(), 0, 1, timeStep),
    curPhase(START),
    execPhase(new EgressMRStartPhase)
{
  collsConstraint.addCollisions(robots(),
      {
        //mc_solver::Collision("RLEG_LINK5", "left_column", 0.1, 0.05, 0.),
        //mc_solver::Collision("RLEG_LINK4", "left_column", 0.1, 0.05, 0.),
        //mc_solver::Collision("RARM_LINK6", "left_column", 0.1, 0.05, 0.),
        //mc_solver::Collision("RARM_LINK5", "left_column", 0.1, 0.05, 0.),
        //mc_solver::Collision("RARM_LINK4", "left_column", 0.1, 0.05, 0.)
        });
  hrp2selfCollisionConstraint.addCollisions(robots(),
      {
      mc_solver::Collision("LLEG_LINK3", "LARM_LINK3", 0.05, 0.01, 0.),
      mc_solver::Collision("LLEG_LINK4", "LARM_LINK3", 0.05, 0.01, 0.)
      });
  mrqpsolver->addConstraintSet(hrp2contactConstraint);
  mrqpsolver->addConstraintSet(hrp2kinematicsConstraint);
  mrqpsolver->addConstraintSet(polarisKinematicsConstraint);
  mrqpsolver->addConstraintSet(hrp2selfCollisionConstraint);
  mrqpsolver->addConstraintSet(collsConstraint);

  mrqpsolver->solver.addTask(hrp2postureTask.get());
  hrp2postureTask->weight(1);
  robots().envIndex = 2;

  mc_rbdyn::Robot& polaris = robots().robots[1];

  robot().mbc->q[0] = {1, 0, 0, 0, 0, 0, 0.76};

  rbd::forwardKinematics(*(robot().mb), *(robot().mbc));
  rbd::forwardVelocity(*(robot().mb), *(robot().mbc));

  egressContacts.emplace_back(robots().robotIndex, 1,
                           robot().surfaces.at("Butthock"),
                           polaris.surfaces.at("lazy_seat"));
  egressContacts.emplace_back(robots().robotIndex, 1,
                           robot().surfaces.at("LFullSole"),
                           polaris.surfaces.at("exit_platform"));
  egressContacts.emplace_back(robots().robotIndex, 1,
                           robot().surfaces.at("RightGripper"),
                           polaris.surfaces.at("bar_wheel"));
  mrqpsolver->setContacts(egressContacts);

  polarisPostureTask.reset(new tasks::qp::PostureTask(mrqpsolver->robots.mbs, 1, mrqpsolver->robots.robots[1].mbc->q, 1.0, 1));
  lazyPostureTask.reset(new tasks::qp::PostureTask(mrqpsolver->robots.mbs, 1, polaris.mbc->q, 0.0, 1000.0));
  std::vector<tasks::qp::JointStiffness> jsv;
  jsv.push_back({static_cast<int>(polaris.jointIdByName("lazy_susan")), 0.1});
  lazyPostureTask->jointsStiffness(robots().mbs, jsv);

  comTask.reset(new mc_tasks::CoMTask(mrqpsolver->robots, mrqpsolver->robots.robotIndex));
  comTask->comTaskSp->stiffness(1.);
  efTask.reset(new mc_tasks::EndEffectorTask("RARM_LINK6", mrqpsolver->robots,
                                             mrqpsolver->robots.robotIndex));

  //collsConstraint.addCollision(robots(), mc_solver::Collision("RLEG_LINK5", "floor", 0.2, 0.15, 0));
  //collsConstraint.addCollision(robots(), mc_solver::Collision("RLEG_LINK5", "floor_step", 0.2, 0.15, 0));
  //collsConstraint.addCollision(robots(), mc_solver::Collision("RLEG_LINK5", "front_plane", 0.3, 0.25, 0));
  std::cout << "MCEgressMRQPController init done" << std::endl;
}

bool MCEgressMRQPController::run()
{
  bool success = MCMRQPController::run();
  if(success)
  {
    bool next = execPhase->run(*this);
    if(next)
    {
      nextPhase();
    }
  }
  else
  {
    std::cout << "Failed to run" << std::endl;
  }
  return success;
}

void MCEgressMRQPController::reset(const ControllerResetData & reset_data)
{
  MCMRQPController::reset(reset_data);
  std::cout << "Enter mr egress reset" << std::endl;
  robot().mbc->zero(*(robot().mb));
  robot().mbc->q = reset_data.q;
  robot().mbc->q[0] = {1, 0, 0, 0, 0, 0, 0.76};
  rbd::forwardKinematics(*(robot().mb), *(robot().mbc));
  rbd::forwardVelocity(*(robot().mb), *(robot().mbc));
  hrp2postureTask->posture(robot().mbc->q);

  resetBasePose();
  resetWheelTransform();
  resetLazyTransform();

  mrqpsolver->solver.addTask(polarisPostureTask.get());
  mrqpsolver->solver.addTask(lazyPostureTask.get());

  mrqpsolver->setContacts(egressContacts);

  mrqpsolver->solver.updateTasksNrVars(robots().mbs);
  mrqpsolver->solver.updateConstrsNrVars(robots().mbs);
  mrqpsolver->solver.updateConstrSize();

  std::cout << "End mr egress reset" << std::endl;
}

void MCEgressMRQPController::addCollision(const mc_solver::Collision& coll)
{
  collsConstraint.addCollisions(robots(), {coll});
}


void MCEgressMRQPController::resetWheelTransform()
{
  sva::PTransformd graspOffset(sva::RotX(-M_PI/2), Eigen::Vector3d(0, 0, 0));
  mc_rbdyn::Robot& polaris = robots().robots[1];
  //Change wheel position
  int chassis_index = polaris.bodyIndexByName("chassis");
  //Do not take into account potential rotation of steering wheel
  int joint_index = polaris.jointIndexByName("adjust_steering_wheel");

  auto gripperSurface = robot().surfaces.at("RightGripper");
  auto wheelSurface = polaris.surfaces.at("bar_wheel");

  sva::PTransformd X_wheel_s = graspOffset*wheelSurface->X_b_s();

  sva::PTransformd X_0_s = gripperSurface->X_0_s(robot(), *(robot().mbc));

  sva::PTransformd X_0_chassis = polaris.mbc->bodyPosW[chassis_index];

  sva::PTransformd X_chassis_wheel = (X_wheel_s).inv()*X_0_s*(X_0_chassis).inv();

  polaris.mb->transform(joint_index, X_chassis_wheel);
  polaris.mbc->zero(*(polaris.mb));

  rbd::forwardKinematics(*(polaris.mb), *(polaris.mbc));
  rbd::forwardVelocity(*(polaris.mb), *(polaris.mbc));
}

void MCEgressMRQPController::resetLazyTransform()
{
  mc_rbdyn::Robot& polaris = robots().robots[1];
  //Change wheel position
  int chassis_index = polaris.bodyIndexByName("chassis");
  //Do not take into account potential rotation of steering wheel
  int joint_index = polaris.jointIndexByName("lazy_susan");

  auto gripperSurface = robot().surfaces.at("Butthock");
  auto wheelSurface = polaris.surfaces.at("lazy_seat");

  sva::PTransformd X_wheel_s = wheelSurface->X_b_s();

  sva::PTransformd X_0_s = gripperSurface->X_0_s(robot(), *(robot().mbc));

  sva::PTransformd X_0_chassis = polaris.mbc->bodyPosW[chassis_index];

  sva::PTransformd X_chassis_wheel = (X_wheel_s).inv()*X_0_s*(X_0_chassis).inv();

  polaris.mb->transform(joint_index, X_chassis_wheel);
  polaris.mbc->zero(*(polaris.mb));

  rbd::forwardKinematics(*(polaris.mb), *(polaris.mbc));
  rbd::forwardVelocity(*(polaris.mb), *(polaris.mbc));
}

void MCEgressMRQPController::resetBasePose()
{
  mc_rbdyn::Robot& polaris = robots().robots[1];
  //Reset freeflyer, compute its position frow wheel and re-set it
  robot().mbc->q[0] = {1., 0., 0., 0., 0., 0., 0.};
  rbd::forwardKinematics(*(robot().mb), *(robot().mbc));
  rbd::forwardVelocity(*(robot().mb), *(robot().mbc));

  auto X_0_w = polaris.surfaces.at("exit_platform")->X_0_s(polaris);
  sva::PTransformd graspOffset(sva::RotZ(M_PI/4), Eigen::Vector3d(-0.1, -0.1, 0));
  auto X_0_s = robot().surfaces.at("LFullSole")->X_0_s(robot());
  auto X_0_base = X_0_s.inv()*(graspOffset*X_0_w);

  const auto quat = Eigen::Quaterniond(X_0_base.rotation()).inverse();
  const Eigen::Vector3d trans(X_0_base.translation());
  std::vector<double> baseQ = {quat.w(), quat.x(), quat.y(), quat.z(),
                               trans.x(), trans.y(), trans.z()};

  robot().mbc->q[0] = baseQ;
  rbd::forwardKinematics(*(robot().mb), *(robot().mbc));
  rbd::forwardVelocity(*(robot().mb), *(robot().mbc));
}

void MCEgressMRQPController::nextPhase()
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
    execPhase.reset(new EgressMoveComSurfPhase("RFullSole", 0.15));
    break;
  case MOVECOMRIGHT:
    curPhase = REPLACELEFTFOOT;
    execPhase.reset(new EgressReplaceLeftFootPhase);
    break;
  case REPLACELEFTFOOT:
    curPhase = MOVECOMFORCELEFT;
    execPhase.reset(new EgressMoveComSurfPhase("LFullSole", 0.10));
    //Use this to lift the rear feet by a maximum of 10cm
    //execPhase.reset(new EgressMoveComForcePhase("LFullSole", 0.10, 0.1));
    break;
  case MOVECOMFORCELEFT:
    curPhase = PUTDOWNRIGHTFOOT;
    execPhase.reset(new EgressPutDownRightFootPhase);
    break;
  case PUTDOWNRIGHTFOOT:
    curPhase = CENTERCOM;
    comTask->comTaskSp->weight(1000.);
    execPhase.reset(new EgressCenterComPhase(0.10));
    break;
  case CENTERCOM:
    comTask->comTaskSp->weight(1.);
    curPhase = OPENGRIPPER;
    execPhase.reset(new EgressOpenRightGripperPhase);
    break;
  case OPENGRIPPER:
    curPhase = REMOVEHAND;
    execPhase.reset(new EgressRemoveRightGripperPhase);
    break;
  default:
    std::cout << "Done" << std::endl;
    break;
  }
}

}
