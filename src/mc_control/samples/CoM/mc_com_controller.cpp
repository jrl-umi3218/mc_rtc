/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_com_controller.h"

#include <mc_rbdyn/Surface.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/SurfaceTransformTask.h>

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

namespace mc_control
{

MCCoMController::MCCoMController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt)
: MCController(robot_module, dt)
{
  qpsolver->addConstraintSet(contactConstraint);
  qpsolver->addConstraintSet(dynamicsConstraint);
  qpsolver->addConstraintSet(selfCollisionConstraint);
  qpsolver->addTask(postureTask);

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
  if(robot().hasSurface("LFullSole") && robot().hasSurface("RFullSole"))
  {
    solver().setContacts(
        {mc_rbdyn::Contact(robots(), "LFullSole", "AllGround"), mc_rbdyn::Contact(robots(), "RFullSole", "AllGround")});
  }
  else if(robot().hasSurface("LeftFoot") && robot().hasSurface("RightFoot"))
  {
    solver().setContacts(
        {mc_rbdyn::Contact(robots(), "LeftFoot", "AllGround"), mc_rbdyn::Contact(robots(), "RightFoot", "AllGround")});
  }
  else
  {
    LOG_ERROR("MCCoMController does not support robot " << robot().name())
    LOG_ERROR_AND_THROW(std::runtime_error, "MCCoMController does not support your robot")
  }
}

sva::PTransformd MCCoMController::anchorFrame(const mc_rbdyn::Robot & robot) const
{
  sva::PTransformd X_0_anchor;
  const auto & contacts = solver().contacts();

  if(contacts.size() == 2)
  {
    sva::PTransformd X_0_c1 = contacts[0].X_0_r2s(robot);
    sva::PTransformd X_0_c2 = contacts[1].X_0_r2s(robot);
    return sva::interpolate(X_0_c1, X_0_c2, 0.5);
  }
  else
  {
    LOG_ERROR_AND_THROW(std::runtime_error,
                        "anchorFrame implementation expects two contacts, " << contacts.size() << " set");
  }
}

} // namespace mc_control
