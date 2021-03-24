/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/api.h>
#include <mc_control/fsm/Controller.h>
#include <mc_control/mc_controller.h>

struct MC_CONTROL_DLLAPI ExternalForcesController : public mc_control::fsm::Controller
{
  ExternalForcesController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  void reset(const mc_control::ControllerResetData & reset_data) override;
  bool run() override;

  void supported_robots(std::vector<std::string> & out) const override
  {
    out = {"jvrc1"};
  }

  inline double t() const noexcept
  {
    return t_;
  }

protected:
  double t_ = 0; ///< Elapsed time since the controller started
};
