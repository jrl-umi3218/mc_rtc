/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_body6d_controller.h"

#include <mc_rtc/logging.h>

#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

/* Note all service calls except for controller switches are implemented in mc_global_controller_services.cpp */

namespace mc_control
{

MCBody6dController::MCBody6dController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt)
: MCController(robot_module, dt)
{
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(dynamicsConstraint);
  solver().addConstraintSet(selfCollisionConstraint);
  solver().addConstraintSet(*compoundJointConstraint);
  solver().addTask(postureTask.get());
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
    LOG_ERROR("MCBody6dController does not support robot " << robot().name())
    LOG_ERROR_AND_THROW(std::runtime_error, "MCBody6dController does not support your robot")
  }

  LOG_SUCCESS("MCBody6dController init done")
  if(robot().hasBody("RARM_LINK7"))
  {
    efTask.reset(new mc_tasks::EndEffectorTask("RARM_LINK7", robots(), robots().robotIndex(), 2.0, 1e5));
  }
  else if(robot().hasBody("r_wrist"))
  {
    efTask.reset(new mc_tasks::EndEffectorTask("r_wrist", robots(), robots().robotIndex(), 2.0, 1e5));
  }
  else
  {
    LOG_ERROR("MCBody6dController does not support robot " << robot().name())
    LOG_ERROR_AND_THROW(std::runtime_error, "MCBody6dController does not support your robot")
  }
  solver().addTask(efTask);
  comTask.reset(new mc_tasks::CoMTask(robots(), robots().robotIndex()));
  solver().addTask(comTask);
}

void MCBody6dController::reset(const ControllerResetData & reset_data)
{
  MCController::reset(reset_data);
  efTask->reset();
  comTask->reset();
}

} // namespace mc_control
