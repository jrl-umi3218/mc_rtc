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
    leftFootSurface = "LFullSole";
    rightFootSurface = "RFullSole";
  }
  else if(robot().hasSurface("LeftFoot") && robot().hasSurface("RightFoot"))
  {
    leftFootSurface = "LeftFoot";
    rightFootSurface = "RightFoot";
  }
  else
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("MCCoMController does not support robot {}", robot().name());
  }
}

void MCCoMController::reset(const ControllerResetData & reset_data)
{
  MCController::reset(reset_data);
  comTask->reset();
  solver().addTask(comTask);
  solver().setContacts({mc_rbdyn::Contact(robots(), leftFootSurface, "AllGround"),
                        mc_rbdyn::Contact(robots(), rightFootSurface, "AllGround")});
  updateAnchorFrame();
}

bool MCCoMController::run()
{
  updateAnchorFrame();
  return MCController::run();
}

void MCCoMController::updateAnchorFrame()
{
  // Project desired CoM in-between foot-sole ankle frames and compute ratio along the line in-beween the two surfaces
  const Eigen::Vector3d & lankle = robot().surfacePose(leftFootSurface).translation();
  const Eigen::Vector3d & rankle = robot().surfacePose(rightFootSurface).translation();
  Eigen::Vector3d t_lankle_com = robot().com() - lankle;
  Eigen::Vector3d t_lankle_rankle = rankle - lankle;
  double d_proj = t_lankle_com.dot(t_lankle_rankle.normalized());
  double leftFootRatio = mc_filter::utils::clamp(d_proj / t_lankle_rankle.norm(), 0., 1.);
  anchorFrame(
      sva::interpolate(robot().surfacePose(leftFootSurface), robot().surfacePose(rightFootSurface), leftFootRatio));
  anchorFrameReal(sva::interpolate(realRobot().surfacePose(leftFootSurface), realRobot().surfacePose(rightFootSurface),
                                   leftFootRatio));
}

} // namespace mc_control
