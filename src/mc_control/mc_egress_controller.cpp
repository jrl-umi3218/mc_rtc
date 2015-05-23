#include <mc_control/mc_egress_controller.h>

namespace mc_control
{

MCEgressController::MCEgressController(const std::string & env_path, const std::string & env_name)
: MCController(env_path, env_name),
  collsConstraint(robots(), timeStep)
{
  /* Recreate the dynamics constrain to remove the damper offset */
  dynamicsConstraint = mc_solver::DynamicsConstraint(qpsolver->robots, 0, timeStep,
                                                     false, {0.1, 0.01, 0.}, 0.5);

  qpsolver->addConstraintSet(contactConstraint);
  qpsolver->addConstraintSet(dynamicsConstraint);
  qpsolver->addConstraintSet(selfCollisionConstraint);
  qpsolver->addConstraintSet(collsConstraint);
  qpsolver->solver.addTask(postureTask.get());

  qpsolver->setContacts({
    mc_rbdyn::Contact(robot().surfaces.at("Butthock"), env().surfaces.at("left_seat")),
    mc_rbdyn::Contact(robot().surfaces.at("LFullSole"), env().surfaces.at("exit_platform")),
    mc_rbdyn::Contact(robot().surfaces.at("LeftThight"), env().surfaces.at("left_seat")),
    mc_rbdyn::Contact(robot().surfaces.at("RightThight"), env().surfaces.at("left_seat")),
  });

  //collsConstraint.addEnvCollision(robots(),
  //  mc_solver::Collision("RARM_LINK6", "wheel_hull", 0.1, 0.05, 0)
  //);

  comTask.reset(new mc_tasks::CoMTask(qpsolver->robots, qpsolver->robots.robotIndex));
  comTask->addToSolver(qpsolver->solver);

  efTask.reset(new mc_tasks::EndEffectorTask("RARM_LINK6", qpsolver->robots, qpsolver->robots.robotIndex));
  efTask->addToSolver(qpsolver->solver);
}

void MCEgressController::reset(const ControllerResetData & reset_data)
{
  MCController::reset(reset_data);
  qpsolver->setContacts({
    mc_rbdyn::Contact(robot().surfaces.at("Butthock"), env().surfaces.at("left_seat")),
    mc_rbdyn::Contact(robot().surfaces.at("LFullSole"), env().surfaces.at("exit_platform")),
    mc_rbdyn::Contact(robot().surfaces.at("LeftThight"), env().surfaces.at("left_seat")),
    mc_rbdyn::Contact(robot().surfaces.at("RightThight"), env().surfaces.at("left_seat")),
  });
  efTask->resetTask(qpsolver->robots, qpsolver->robots.robotIndex);
  comTask->resetTask(qpsolver->robots, qpsolver->robots.robotIndex);
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
  sva::PTransformd dtr(m, v);
  efTask->add_ef_pose(dtr);
  return true;
}

bool MCEgressController::move_com(const Eigen::Vector3d & v)
{
  comTask->move_com(v);
  return true;
}

}
