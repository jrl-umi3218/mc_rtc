/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/api.h>
#include <mc_control/fsm/Controller.h>
#include <mc_control/mc_controller.h>

struct MC_CONTROL_DLLAPI FSMController : public mc_control::fsm::Controller
{
  FSMController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  void supported_robots(std::vector<std::string> & out) const override
  {
    out = {"jvrc1"};
  }
};
