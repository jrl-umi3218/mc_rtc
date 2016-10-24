#include "mc_egress_controller.h"

#include <mc_rtc/logging.h>

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Surface.h>

#include "mc_egress_phases.cpp"

#include <array>

namespace mc_control
{

MCEgressController::MCEgressController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt)
: MCController({robot_module, mc_rbdyn::RobotLoader::get_robot_module("env", std::string(mc_rtc::HRP2_DRC_DESCRIPTION_PATH), std::string("polaris_ranger"))}, dt),
  collsConstraint(robots(), 0, 1, timeStep),
  phase(START), phaseExec(new EgressStartPhase)
  //phase(ROTATEBODY), phaseExec(new EgressRotateBodyPhase)
{
  /* Recreate the kinematics/dynamics constraints to lower the damper offset */
  std::array<double, 3> damper = {{0.1, 0.01, 0.00}};
  kinematicsConstraint = mc_solver::KinematicsConstraint(robots(), 0, timeStep, damper, 0.5);
  damper = {{0.1, 0.01, 0.1}};
  dynamicsConstraint = mc_solver::DynamicsConstraint(robots(), 0, timeStep, damper, 0.5);

  qpsolver->addConstraintSet(contactConstraint);
  //qpsolver->addConstraintSet(dynamicsConstraint);
  qpsolver->addConstraintSet(kinematicsConstraint);
  qpsolver->addConstraintSet(selfCollisionConstraint);
  qpsolver->addConstraintSet(collsConstraint);
  qpsolver->addTask(postureTask.get());

  qpsolver->setContacts({
    mc_rbdyn::Contact(robots(), "Butthock", "left_seat"),
    mc_rbdyn::Contact(robots(), "LFullSole", "exit_platform"),
    mc_rbdyn::Contact(robots(), "RightGripper", "bar_wheel")
  });

  comTask.reset(new mc_tasks::CoMTask(robots(), robots().robotIndex()));
  solver().addTask(comTask);
  solver().removeTask(comTask);

  efTask.reset(new mc_tasks::EndEffectorTask("RARM_LINK6", robots(), robots().robotIndex()));
  solver().addTask(efTask);
  solver().removeTask(efTask);
  LOG_SUCCESS("MCEgressController init done")
}

void MCEgressController::reset(const ControllerResetData & reset_data)
{
  MCController::reset(reset_data);
  resetBasePose();
  qpsolver->setContacts({
    mc_rbdyn::Contact(robots(), "Butthock", "left_seat"),
    mc_rbdyn::Contact(robots(), "LFullSole", "exit_platform"),
    mc_rbdyn::Contact(robots(), "RightGripper", "bar_wheel")
  });
  efTask->reset();
  comTask->reset();
}

void MCEgressController::resetBasePose()
{
  mc_rbdyn::Robot& polaris = robots().robot(1);
  //Reset freeflyer, compute its position frow wheel and re-set it
  robot().mbc().q[0] = {1., 0., 0., 0., 0., 0., 0.};
  rbd::forwardKinematics(robot().mb(), robot().mbc());
  rbd::forwardVelocity(robot().mb(), robot().mbc());

  //unsigned int steer_i = polaris.bodyIndexByName("steering_wheel");
  //sva::PTransformd X_0_w = polaris.mbc().bodyPosW[steer_i];
  //const auto & gripperSurface = robot().surface("RightGripper");
  //sva::PTransformd X_0_s = gripperSurface.X_0_s(robot(), robot().mbc());
  //sva::PTransformd graspOffset(sva::RotX(-M_PI/2), Eigen::Vector3d(0., 0., 0.));
  //sva::PTransformd X_0_base = X_0_s.inv()*(graspOffset*X_0_w);
  //sva::PTransformd X_0_base = X_0_s.inv()*X_0_w;
  sva::PTransformd X_0_w = polaris.surface("exit_platform").X_0_s(polaris);
  sva::PTransformd X_0_s = robot().surface("LFullSole").X_0_s(robot());
  sva::PTransformd X_0_base = X_0_s.inv()*X_0_w;

  const auto quat = Eigen::Quaterniond(X_0_base.rotation()).inverse();
  const Eigen::Vector3d trans(X_0_base.translation());
  std::vector<double> baseQ = {quat.w(), quat.x(), quat.y(), quat.z(),
                               trans.x(), trans.y(), trans.z()};

  robot().mbc().q[0] = baseQ;
  rbd::forwardKinematics(robot().mb(), robot().mbc());
  rbd::forwardVelocity(robot().mb(), robot().mbc());
}

bool MCEgressController::run()
{
  bool ret = MCController::run();
  if(ret)
  {
    bool next = phaseExec->run(*this);
    if(next)
    {
      play_next_stance();
    }
  }
  return ret;
}

bool MCEgressController::change_ef(const std::string & ef_name)
{
  if(robot().hasBody(ef_name))
  {
    solver().removeTask(efTask);
    postureTask->posture(robot().mbc().q);
    efTask.reset(new mc_tasks::EndEffectorTask(ef_name, robots(), robots().robotIndex()));
    solver().addTask(efTask);
    return true;
  }
  else
  {
    LOG_ERROR("Invalid link name: " << ef_name << ", control unchanged")
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

bool MCEgressController::play_next_stance()
{
  bool new_phase = true;
  switch(phase)
  {
    case START:
      phase = MOVEFOOTINSIDE;
      phaseExec.reset(new EgressMoveFootInsidePhase);
      break;
    case MOVEFOOTINSIDE:
      phase = REMOVEHAND;
      phaseExec.reset(new EgressRemoveHandPhase);
      break;
    case REMOVEHAND:
      phase = ROTATEBODY;
      phaseExec.reset(new EgressRotateBodyPhase);
      break;
    case ROTATEBODY:
      phase = CORRECTLFOOT;
      phaseExec.reset(new EgressCorrectLeftFootPhase);
      break;
    case CORRECTLFOOT:
      phase = CORRECTBODY;
      phaseExec.reset(new EgressRotateBodyPhase);
      break;
    case CORRECTBODY:
      phase = MOVEFOOTOUT;
      phaseExec.reset(new EgressMoveFootOutPhase);
      break;
    case MOVEFOOTOUT:
      phase = STANDUP;
      phaseExec.reset(new EgressStandupPhase);
      break;
    case CORRECTRFOOT:
      phase = CORRECTBODY;
      break;
    default:
      new_phase = false;
      break;
  }
  return new_phase;
}

std::vector<std::string> MCEgressController::supported_robots() const
{
  return {"hrp2_drc"};
}

}
