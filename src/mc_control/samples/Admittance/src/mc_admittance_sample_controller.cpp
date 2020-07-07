/*
 * Copyright 2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_admittance_sample_controller.h"

AdmittanceSampleController::AdmittanceSampleController(mc_rbdyn::RobotModulePtr rm,
                                                       double dt,
                                                       const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
}

void AdmittanceSampleController::reset(const mc_control::ControllerResetData & reset_data)
{
  Controller::reset(reset_data);
  // Update anchor frame for the KinematicInertial observer
  anchorFrame(sva::interpolate(robot().surfacePose("RightFoot"), robot().surfacePose("LeftFoot"), 0.5));
  anchorFrameReal(sva::interpolate(realRobot().surfacePose("RightFoot"), realRobot().surfacePose("LeftFoot"), 0.5));
}

bool AdmittanceSampleController::run()
{
  // Update anchor frame for the KinematicInertial observer
  anchorFrame(sva::interpolate(robot().surfacePose("RightFoot"), robot().surfacePose("LeftFoot"), 0.5));
  anchorFrameReal(sva::interpolate(realRobot().surfacePose("RightFoot"), realRobot().surfacePose("LeftFoot"), 0.5));
  return Controller::run();
}
