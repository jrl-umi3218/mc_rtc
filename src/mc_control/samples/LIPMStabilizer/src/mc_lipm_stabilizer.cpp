/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_lipm_stabilizer.h"

LIPMStabilizerController::LIPMStabilizerController(mc_rbdyn::RobotModulePtr rm,
                                                   double dt,
                                                   const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
}

bool LIPMStabilizerController::run()
{
  return mc_control::fsm::Controller::run();
}

void LIPMStabilizerController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}

sva::PTransformd LIPMStabilizerController::anchorFrame(const mc_rbdyn::Robot & robot) const
{
  // XXX should be querying anchor frame from Stabilizer instead
  return sva::interpolate(robot.surfacePose("LeftFootCenter"), robot.surfacePose("RightFootCenter"), 0.5);
}
