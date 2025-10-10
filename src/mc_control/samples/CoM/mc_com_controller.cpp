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

MCCoMController::MCCoMController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt, Backend backend)
: MCController(robot_module, dt, backend)
{
  qpsolver->addConstraintSet(contactConstraint);
  qpsolver->addConstraintSet(selfCollisionConstraint);
  qpsolver->addConstraintSet(*compoundJointConstraint);
  qpsolver->addTask(postureTask);

  comTask.reset(new mc_tasks::CoMTask(robots(), robots().robotIndex()));
  postureTask->stiffness(1);
  postureTask->weight(1);
  comTask->weight(1000);

  if(robot().hasSurface("LFullSole") && robot().hasSurface("RFullSole"))
  {
    surfaces_.push_back("LFullSole");
    surfaces_.push_back("RFullSole");
  }
  else if(robot().hasSurface("LeftFoot") && robot().hasSurface("RightFoot"))
  {
    surfaces_.push_back("LeftFoot");
    surfaces_.push_back("RightFoot");
  }
  else if(robot().hasSurface("MobileBase")) { surfaces_.push_back("MobileBase"); }
  else
  {
    mc_rtc::log::error_and_throw("MCCoMController does not support robot {}", robot().name());
  }

  if(surfaces_.size() == 2)
  {
    datastore().make_call(
        "KinematicAnchorFrame::" + robot().name(), [this](const mc_rbdyn::Robot & robot)
        { return sva::interpolate(robot.surfacePose(surfaces_.front()), robot.surfacePose(surfaces_.back()), 0.5); });
  }
  else
  {
    datastore().make_call("KinematicAnchorFrame::" + robot().name(),
                          [this](const mc_rbdyn::Robot & robot) { return robot.surfacePose(surfaces_.front()); });
  }
}

void MCCoMController::reset(const ControllerResetData & reset_data)
{
  MCController::reset(reset_data);
  comTask->reset();
  solver().addTask(comTask);
  qpsolver->addConstraintSet(dynamicsConstraint);
  for(const auto & surface : surfaces_) { addContact(Contact{env().name(), robot().name(), "AllGround", surface}); }
}

} // namespace mc_control

MULTI_CONTROLLERS_CONSTRUCTOR("CoM",
                              mc_control::MCCoMController(rm, dt, mc_control::MCController::Backend::Tasks),
                              "CoM_TVM",
                              mc_control::MCCoMController(rm, dt, mc_control::MCController::Backend::TVM))
