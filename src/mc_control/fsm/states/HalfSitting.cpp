/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/states/HalfSitting.h>

namespace mc_control
{

namespace fsm
{

void HalfSittingState::configure(const mc_rtc::Configuration & config)
{
  if(config.has("stiffness"))
  {
    has_stiffness_ = true;
    stiffness_ = config("stiffness");
  }
  if(config.has("eval"))
  {
    has_eval_ = true;
    eval_threshold_ = config("eval");
  }
}

void HalfSittingState::start(Controller & ctl)
{
  auto postureTask = ctl.getPostureTask(ctl.robot().name());
  default_stiffness_ = postureTask->stiffness();
  if(has_stiffness_)
  {
    postureTask->stiffness(stiffness_);
  }
  /* Set the halfSitPose in posture Task */
  const auto & halfSit = ctl.robot().module().stance();
  const auto & ref_joint_order = ctl.robot().refJointOrder();
  auto posture = postureTask->posture();
  for(unsigned int i = 0; i < ref_joint_order.size(); ++i)
  {
    if(ctl.robot().hasJoint(ref_joint_order[i]))
    {
      posture[ctl.robot().jointIndexByName(ref_joint_order[i])] = halfSit.at(ref_joint_order[i]);
    }
  }
  postureTask->posture(posture);
}

bool HalfSittingState::run(Controller & ctl)
{
  auto postureTask = ctl.getPostureTask(ctl.robot().name());
  if(!has_eval_ || postureTask->eval().norm() < eval_threshold_)
  {
    postureTask->stiffness(default_stiffness_);
    output("OK");
    return true;
  }
  return false;
}

} // namespace fsm

} // namespace mc_control

EXPORT_SINGLE_STATE("HalfSitting", mc_control::fsm::HalfSittingState)
