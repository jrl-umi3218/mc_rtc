#include "mc_com_controller.h"
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <mc_rbdyn/Surface.h>

#include <mc_tasks/SurfaceTransformTask.h>

#include <mc_rtc/logging.h>

namespace mc_control
{

MCCoMController::MCCoMController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt)
: MCController(robot_module, dt)
{
  qpsolver->addConstraintSet(contactConstraint);
  qpsolver->addConstraintSet(dynamicsConstraint);
  //qpsolver->addConstraintSet(kinematicsConstraint);
  qpsolver->addConstraintSet(selfCollisionConstraint);
  auto pt = std::make_shared<mc_tasks::PostureTask>(solver(), 0, 2.0, 1.0);
  solver().addTask(pt);
  //qpsolver->addTask(postureTask.get());

  comTask.reset(new mc_tasks::CoMTask(robots(), robots().robotIndex()));
  postureTask->stiffness(1);
  postureTask->weight(1);
  comTask->weight(1000);
}

void MCCoMController::reset(const ControllerResetData & reset_data)
{
  MCController::reset(reset_data);
  comTask->reset();
  solver().addTask(comTask);
  if(robot().name() == "hrp2_drc")
  {
    mc_rbdyn::Contact c{robots(), "LeftFingers", "AllGround"};
    auto id = c.contactId(robots());
    Eigen::Matrix6d dof = Eigen::Matrix6d::Identity();
    dof(5,5) = 0;
    qpsolver->setContacts({
      mc_rbdyn::Contact(robots(), "LFullSole", "AllGround"),
      mc_rbdyn::Contact(robots(), "RFullSole", "AllGround"),
      c
    });
    contactConstraint.contactConstr->addDofContact(id, dof);
    contactConstraint.contactConstr->updateDofContacts();
  }
  else if(robot().name() == "hrp4")
  {
    mc_rbdyn::Contact c{robots(), "LeftHand", "AllGround"};
    auto id = c.contactId(robots());
    Eigen::Matrix6d dof = Eigen::Matrix6d::Identity();
    dof(5,5) = 0;
    contactConstraint.contactConstr->addDofContact(id, dof);
    contactConstraint.contactConstr->updateDofContacts();
    qpsolver->setContacts({
      mc_rbdyn::Contact(robots(), "LeftFoot", "AllGround"),
      mc_rbdyn::Contact(robots(), "RightFoot", "AllGround"),
      //c
    });
    auto t = std::make_shared<mc_tasks::SurfaceTransformTask>("LeftHand", robots(), 0);
    //t->target(sva::PTransformd(Eigen::Vector3d{0, 0, 0.1}) * t->target());
    solver().addTask(t);
  }
  else
  {
    LOG_ERROR("MCCoMController does not support robot " << robot().name())
    LOG_ERROR_AND_THROW(std::runtime_error, "MCCoMController does not support your robot")
  }
}

bool MCCoMController::move_com(const Eigen::Vector3d & v)
{
  comTask->move_com(v);
  return true;
}

}
