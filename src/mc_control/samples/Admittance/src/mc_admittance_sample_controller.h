/*
 * Copyright 2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/api.h>
#include <mc_control/fsm/Controller.h>
#include <mc_control/mc_controller.h>

struct MC_CONTROL_DLLAPI AdmittanceSampleController : public mc_control::fsm::Controller
{
  AdmittanceSampleController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  void reset(const mc_control::ControllerResetData & reset_data) override;
  bool run() override;

  void supported_robots(std::vector<std::string> & out) const override
  {
    out = {"jvrc1"};
  }

protected:
  double t_ = 0; ///< Elapsed time since the controller started
};
