/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_com_controller.h"

#include <mc_filter/utils/clamp.h>
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
  qpsolver->addConstraintSet(*compoundJointConstraint);
  qpsolver->addTask(postureTask);

  comTask.reset(new mc_tasks::CoMTask(robots(), robots().robotIndex()));
  postureTask->stiffness(1);
  postureTask->weight(1);
  comTask->weight(1000);

  if(robot().hasSurface("LFullSole") && robot().hasSurface("RFullSole"))
  {
    leftFootSurface_ = "LFullSole";
    rightFootSurface_ = "RFullSole";
  }
  else if(robot().hasSurface("LeftFoot") && robot().hasSurface("RightFoot"))
  {
    leftFootSurface_ = "LeftFoot";
    rightFootSurface_ = "RightFoot";
  }
  else
  {
    mc_rtc::log::error_and_throw("MCCoMController does not support robot {}", robot().name());
  }

  datastore().make_call("KinematicAnchorFrame::" + robot().name(), [this](const mc_rbdyn::Robot & robot) {
    return sva::interpolate(robot.surfacePose(leftFootSurface_), robot.surfacePose(rightFootSurface_), 0.5);
  });
}

void MCCoMController::reset(const ControllerResetData & reset_data)
{
  MCController::reset(reset_data);
  comTask->reset();
  solver().addTask(comTask);
  solver().setContacts(
      {mc_rbdyn::Contact(robots(), env().robotIndex(), robot().robotIndex(), "AllGround", leftFootSurface_),
       mc_rbdyn::Contact(robots(), env().robotIndex(), robot().robotIndex(), "AllGround", rightFootSurface_)});
}

} // namespace mc_control
