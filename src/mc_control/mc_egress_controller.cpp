#include <mc_control/mc_egress_controller.h>

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

namespace mc_control
{

MCEgressController::MCEgressController(const std::string & env_path, const std::string & env_name)
: MCController(env_path, env_name),
  collsConstraint(robots(), 0, 1, timeStep),
  need_setup(true),
  //phase(MOVEFOOTINSIDE)
  phase(ROTATEBODY)
{
  /* Recreate the dynamics constrain to remove the damper offset */
  kinematicsConstraint = mc_solver::KinematicsConstraint(qpsolver->robots, 0, timeStep,
                                                     false, {0.1, 0.01, 0.1}, 0.5);
  dynamicsConstraint = mc_solver::DynamicsConstraint(qpsolver->robots, 0, timeStep,
                                                     false, {0.1, 0.01, 0.1}, 0.5);

  qpsolver->addConstraintSet(contactConstraint);
  //qpsolver->addConstraintSet(dynamicsConstraint);
  qpsolver->addConstraintSet(kinematicsConstraint);
  qpsolver->addConstraintSet(selfCollisionConstraint);
  qpsolver->addConstraintSet(collsConstraint);
  qpsolver->solver.addTask(postureTask.get());

  qpsolver->setContacts({
    mc_rbdyn::Contact(robot().surfaces.at("Butthock"), env().surfaces.at("left_seat")),
    mc_rbdyn::Contact(robot().surfaces.at("LFullSole"), env().surfaces.at("exit_platform")),
    mc_rbdyn::Contact(robot().surfaces.at("LeftThight"), env().surfaces.at("left_seat")),
    mc_rbdyn::Contact(robot().surfaces.at("RightThight"), env().surfaces.at("left_seat")),
  });

  comTask.reset(new mc_tasks::CoMTask(qpsolver->robots, qpsolver->robots.robotIndex));
  comTask->addToSolver(qpsolver->solver);
  comTask->removeFromSolver(qpsolver->solver);

  efTask.reset(new mc_tasks::EndEffectorTask("RARM_LINK6", qpsolver->robots, qpsolver->robots.robotIndex));
  efTask->addToSolver(qpsolver->solver);
  efTask->removeFromSolver(qpsolver->solver);
}

void MCEgressController::reset(const ControllerResetData & reset_data)
{
  MCController::reset(reset_data);
  resetBasePose();
  qpsolver->setContacts({
    mc_rbdyn::Contact(robot().surfaces.at("Butthock"), env().surfaces.at("left_seat")),
    mc_rbdyn::Contact(robot().surfaces.at("LFullSole"), env().surfaces.at("exit_platform")),
    mc_rbdyn::Contact(robot().surfaces.at("LeftThight"), env().surfaces.at("left_seat")),
    mc_rbdyn::Contact(robot().surfaces.at("RightThight"), env().surfaces.at("left_seat")),
  });
  efTask->resetTask(qpsolver->robots, qpsolver->robots.robotIndex);
  comTask->resetTask(qpsolver->robots, qpsolver->robots.robotIndex);
}

void MCEgressController::resetBasePose()
{
  mc_rbdyn::Robot& polaris = robots().robots[1];
  //Reset freeflyer, compute its position frow wheel and re-set it
  robot().mbc->q[0] = {1., 0., 0., 0., 0., 0., 0.};
  rbd::forwardKinematics(*(robot().mb), *(robot().mbc));
  rbd::forwardVelocity(*(robot().mb), *(robot().mbc));

  int steer_i = polaris.bodyIndexByName("steering_wheel");
  sva::PTransformd X_0_w = polaris.mbc->bodyPosW[steer_i];
  auto gripperSurface = robot().surfaces.at("RightGripper");
  sva::PTransformd X_0_s = gripperSurface->X_0_s(robot(), *(robot().mbc));
  sva::PTransformd graspOffset(sva::RotX(-M_PI/2), Eigen::Vector3d(0., 0., 0.));
  sva::PTransformd X_0_base = X_0_s.inv()*(graspOffset*X_0_w);
  //sva::PTransformd X_0_base = X_0_s.inv()*X_0_w;

  const auto quat = Eigen::Quaterniond(X_0_base.rotation()).inverse();
  const Eigen::Vector3d trans(X_0_base.translation());
  std::vector<double> baseQ = {quat.w(), quat.x(), quat.y(), quat.z(),
                               trans.x(), trans.y(), trans.z()};

  robot().mbc->q[0] = baseQ;
  rbd::forwardKinematics(*(robot().mb), *(robot().mbc));
  rbd::forwardVelocity(*(robot().mb), *(robot().mbc));
}

bool MCEgressController::run()
{
  if(need_setup)
  {
    need_setup = false;
    switch(phase)
    {
      case MOVEFOOTINSIDE:
        setup_movefootinside();
        break;
      case LIFTBODY:
        setup_liftbody();
        break;
      case ROTATEBODY:
        setup_rotatebody();
        break;
      default:
        break;
    }
  }
  return MCController::run();
}

bool MCEgressController::change_ef(const std::string & ef_name)
{
  if(robot().hasBody(ef_name))
  {
    efTask->removeFromSolver(qpsolver->solver);
    postureTask->posture(robot().mbc->q);
    efTask.reset(new mc_tasks::EndEffectorTask(ef_name, qpsolver->robots, qpsolver->robots.robotIndex));
    efTask->addToSolver(qpsolver->solver);
    return true;
  }
  else
  {
    std::cerr << "Invalid link name: " << ef_name << ", control unchanged" << std::endl;
    return false;
  }
}

bool MCEgressController::move_ef(const Eigen::Vector3d & v, const Eigen::Matrix3d & m)
{
  sva::PTransformd cur_pos = efTask->get_ef_pose();
  sva::PTransformd dtr(m, v);
  efTask->set_ef_pose(dtr*cur_pos);
  return true;
}

bool MCEgressController::move_com(const Eigen::Vector3d & v)
{
  comTask->move_com(v);
  return true;
}

bool MCEgressController::next_phase()
{
  bool new_phase = true;
  switch(phase)
  {
    case MOVEFOOTINSIDE:
      phase = LIFTBODY;
      break;
    case LIFTBODY:
      phase = ROTATEBODY;
      break;
    case ROTATEBODY:
      phase = PUTDOWNBODY;
      break;
    case PUTDOWNBODY:
      phase = FREEHAND;
      break;
    case FREEHAND:
      phase = SAFEHAND;
      break;
    case SAFEHAND:
      phase = MOVEFOOTOUTSIDE;
      break;
    default:
      new_phase = false;
      break;
  }
  need_setup = new_phase;
  return need_setup;
}

void MCEgressController::setup_movefootinside()
{
  std::cout << "MOVEFOOTINSIDE" << std::endl;
  qpsolver->setContacts({
    mc_rbdyn::Contact(robot().surfaces.at("Butthock"), env().surfaces.at("left_seat")),
    mc_rbdyn::Contact(robot().surfaces.at("LFullSole"), env().surfaces.at("exit_platform")),
    mc_rbdyn::Contact(robot().surfaces.at("LeftThight"), env().surfaces.at("left_seat")),
    mc_rbdyn::Contact(robot().surfaces.at("RightGripper"), env().surfaces.at("bar_wheel"))
  });
  efTask.reset(new mc_tasks::EndEffectorTask("RLEG_LINK5", qpsolver->robots, qpsolver->robots.robotIndex));
  efTask->addToSolver(qpsolver->solver);
}

void MCEgressController::setup_rotatebody()
{
  std::cout << "ROTATEBODY" << std::endl;
  qpsolver->setContacts({
    mc_rbdyn::Contact(robot().surfaces.at("LFullSole"), env().surfaces.at("exit_platform")),
    mc_rbdyn::Contact(robot().surfaces.at("RFullSole"), env().surfaces.at("left_floor")),
  });
  comTask->removeFromSolver(qpsolver->solver);
  efTask.reset(new mc_tasks::EndEffectorTask("BODY", qpsolver->robots, qpsolver->robots.robotIndex));
  efTask->addToSolver(qpsolver->solver);
  postureTask->posture(robot().mbc->q);
}

void MCEgressController::setup_liftbody()
{
  std::cout << "LIFTBODY" << std::endl;
  qpsolver->setContacts({
    mc_rbdyn::Contact(robot().surfaces.at("LFullSole"), env().surfaces.at("exit_platform")),
    mc_rbdyn::Contact(robot().surfaces.at("RFullSole"), env().surfaces.at("left_floor")),
    mc_rbdyn::Contact(robot().surfaces.at("RightGripper"), env().surfaces.at("bar_wheel"))
  });
  efTask->removeFromSolver(qpsolver->solver);
  postureTask->posture(robot().mbc->q);
  comTask->resetTask(robots(), 0);
  comTask->addToSolver(qpsolver->solver);
}

}
