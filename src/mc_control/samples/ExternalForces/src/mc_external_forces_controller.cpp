/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_external_forces_controller.h"

ExternalForcesController::ExternalForcesController(mc_rbdyn::RobotModulePtr rm,
                                                   double dt,
                                                   const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
  datastore().make_call("KinematicAnchorFrame::" + robot().name(), [](const mc_rbdyn::Robot & robot) {
    return sva::interpolate(robot.surfacePose("LeftFoot"), robot.surfacePose("RightFoot"), 0.5);
  });
}

void ExternalForcesController::reset(const mc_control::ControllerResetData & reset_data)
{
  Controller::reset(reset_data);

  auto handForceConfig = mc_rtc::gui::ForceConfig(mc_rtc::gui::Color::Red);
  handForceConfig.force_scale *= 10;
  gui()->addElement({"Forces"},
                    mc_rtc::gui::Force(
                        "LeftGripperForce", handForceConfig, [this]() { return robot().surfaceWrench("LeftGripper"); },
                        [this]() { return robot().surfacePose("LeftGripper"); }),
                    mc_rtc::gui::Force(
                        "RightGripperForce", handForceConfig,
                        [this]() { return robot().surfaceWrench("RightGripper"); },
                        [this]() { return robot().surfacePose("RightGripper"); }));

  gui()->addPlot("Surface wrenches", mc_rtc::gui::plot::X("t", [this]() { return t_; }),
                 mc_rtc::gui::plot::Y(
                     "LeftGripperForceZ", [this]() { return robot().surfaceWrench("LeftGripper").force().z(); },
                     mc_rtc::gui::Color::Red),
                 mc_rtc::gui::plot::Y(
                     "RightGripperForceZ", [this]() { return robot().surfaceWrench("RightGripper").force().z(); },
                     mc_rtc::gui::Color::Red));
}

bool ExternalForcesController::run()
{
  t_ += timeStep;
  return Controller::run();
}
