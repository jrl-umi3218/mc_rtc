/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/CompletionCriteria.h>
#include <mc_control/fsm/State.h>
#include <mc_tasks/ImpedanceTask.h>
#include <mc_tasks/lipm_stabilizer/StabilizerTask.h>

/*
 * Update target external forces of hands
 */
struct UpdateForces : mc_control::fsm::State
{

  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  mc_rtc::Configuration config_;

  mc_tasks::lipm_stabilizer::StabilizerTask * stabilizerTask_;

  std::vector<mc_tasks::force::ImpedanceTask *> impedanceTasks_;

  double startTime_;
};
