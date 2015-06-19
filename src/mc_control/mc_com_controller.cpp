#include <mc_control/mc_com_controller.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <mc_rbdyn/Surface.h>

namespace mc_control
{

MCCoMController::MCCoMController()
{
  sva::PTransformd leftFootSurfTf = robot().surfaces["LFullSole"]->X_0_s(robot());
  auto q = robot().mbc->q;
  q[0] = {1, 0, 0, 0, 0, 0, -leftFootSurfTf.translation().z()};
  robot().mbc->q = q;
  rbd::forwardKinematics(*(robot().mb), *(robot().mbc));
  rbd::forwardVelocity(*(robot().mb), *(robot().mbc));

  qpsolver->addConstraintSet(contactConstraint);
  qpsolver->addConstraintSet(dynamicsConstraint);
  qpsolver->addConstraintSet(selfCollisionConstraint);
  qpsolver->solver.addTask(postureTask.get());
  qpsolver->setContacts({
    mc_rbdyn::Contact(robots(), "LFullSole", "AllGround"),
    mc_rbdyn::Contact(robots(), "RFullSole", "AllGround")
  });

  comTask.reset(new mc_tasks::CoMTask(qpsolver->robots, qpsolver->robots.robotIndex));
  comTask->addToSolver(qpsolver->solver);
}

void MCCoMController::reset(const ControllerResetData & reset_data)
{
  MCController::reset(reset_data);
  if(reset_data.contacts.size())
  {
    qpsolver->setContacts(reset_data.contacts);
  }
  else
  {
    qpsolver->setContacts({
      mc_rbdyn::Contact(robots(), "LFullSole", "AllGround"),
      mc_rbdyn::Contact(robots(), "RFullSole", "AllGround")
    });
  }
  comTask->resetTask(qpsolver->robots, qpsolver->robots.robotIndex);
}

bool MCCoMController::move_com(const Eigen::Vector3d & v)
{
  comTask->move_com(v);
  return true;
}

}
