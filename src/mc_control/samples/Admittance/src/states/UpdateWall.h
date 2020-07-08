/*
 * Copyright 2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/CompletionCriteria.h>
#include <mc_control/fsm/State.h>

/*
 * Move an environment robot pose according to another robot's surface
 */
struct UpdateWall : mc_control::fsm::State
{

  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  mc_rtc::Configuration config_;
};
