/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_fsm_controller.h"

FSMController::FSMController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
}

void FSMController::reset(const mc_control::ControllerResetData & data)
{
  mc_control::fsm::Controller::reset(data);
  auto check_contact = [this](const std::string & r1, const std::string & r2, const std::string & s1,
                              const std::string & s2) {
    if(!hasContact({r1, r2, s1, s2}))
    {
      mc_rtc::log::error_and_throw("[FSM] Expected to find {}::{}/{}::{} contact", r1, s1, r2, s2);
    }
    if(!hasContact({r2, r1, s2, s1}))
    {
      mc_rtc::log::error_and_throw("[FSM] Expected to find {}::{}/{}::{} contact", r2, s2, r1, s1);
    }
  };
  check_contact("jvrc1", "ground", "LeftFoot", "AllGround");
}
