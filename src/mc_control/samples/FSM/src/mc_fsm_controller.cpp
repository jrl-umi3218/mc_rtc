/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_fsm_controller.h"

FSMController::FSMController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
}

bool FSMController::run()
{
  return mc_control::fsm::Controller::run();
}

void FSMController::reset(const mc_control::ControllerResetData & reset_data)
{
  if(robot().name() != "jvrc1")
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "The FSM sample controller only supports the JVRC1 robot");
  }
  mc_control::fsm::Controller::reset(reset_data);
}
