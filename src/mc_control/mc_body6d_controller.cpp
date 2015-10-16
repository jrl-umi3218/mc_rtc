#include <mc_control/mc_body6d_controller.h>

#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

/* Note all service calls except for controller switches are implemented in mc_global_controller_services.cpp */

namespace mc_control
{

MCBody6dController::MCBody6dController()
: MCController()
{
  qpsolver->addConstraintSet(contactConstraint);
  qpsolver->addConstraintSet(dynamicsConstraint);
  qpsolver->addConstraintSet(selfCollisionConstraint);
  qpsolver->solver.addTask(postureTask.get());
  qpsolver->setContacts({
    mc_rbdyn::Contact(robots(), "LFullSole", "AllGround"),
    mc_rbdyn::Contact(robots(), "RFullSole", "AllGround")
  });

  std::cout << "MCBody6dController init done" << std::endl;
  efTask.reset(new mc_tasks::EndEffectorTask("RARM_LINK7", qpsolver->robots, qpsolver->robots.robotIndex()));
  efTask->addToSolver(qpsolver->solver);
  comTask.reset(new mc_tasks::CoMTask(qpsolver->robots, qpsolver->robots.robotIndex()));
  comTask->addToSolver(qpsolver->solver);
}

void MCBody6dController::reset(const ControllerResetData & reset_data)
{
  MCController::reset(reset_data);
  qpsolver->setContacts({
    mc_rbdyn::Contact(robots(), "LFullSole", "AllGround"),
    mc_rbdyn::Contact(robots(), "RFullSole", "AllGround")
  });
  efTask->resetTask(qpsolver->robots, qpsolver->robots.robotIndex());
  comTask->resetTask(qpsolver->robots, qpsolver->robots.robotIndex());
}


bool MCBody6dController::change_ef(const std::string & ef_name)
{
  if(robot().hasBody(ef_name))
  {
    efTask->removeFromSolver(qpsolver->solver);
    postureTask->posture(robot().mbc().q);
    efTask.reset(new mc_tasks::EndEffectorTask(ef_name, qpsolver->robots, qpsolver->robots.robotIndex()));
    efTask->addToSolver(qpsolver->solver);
    return true;
  }
  else
  {
    std::cerr << "Invalid link name: " << ef_name << ", control unchanged" << std::endl;
    return false;
  }
}

bool MCBody6dController::translate_ef(const Eigen::Vector3d & t)
{
  sva::PTransformd dtr(Eigen::Matrix3d::Identity(), t);
  efTask->add_ef_pose(dtr);
  return true;
}

bool MCBody6dController::rotate_ef(const Eigen::Matrix3d & m)
{
  sva::PTransformd dtr(m, Eigen::Vector3d(0,0,0));
  efTask->add_ef_pose(dtr);
  return true;
}

}
