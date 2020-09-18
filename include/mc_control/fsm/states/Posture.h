/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/CompletionCriteria.h>
#include <mc_control/fsm/State.h>
#include <mc_tasks/PostureTask.h>

namespace mc_control
{

namespace fsm
{

/** Change the global posture */
struct MC_CONTROL_FSM_STATE_DLLAPI PostureState : State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(Controller &) override;

  bool run(Controller &) override;

  void teardown(Controller &) override;

protected:
  mc_rtc::Configuration config_;
  std::string robot_ = "";
  mc_control::CompletionCriteria crit_;
  bool hasCompletion_ = false;
  mc_rtc::Configuration completion_;
};

} // namespace fsm

} // namespace mc_control
