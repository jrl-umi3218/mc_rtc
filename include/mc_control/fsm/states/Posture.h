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
  std::shared_ptr<mc_tasks::PostureTask> postureTask_;
  mc_rtc::Configuration postureTaskConfig_;
  bool has_postureTask_ = false;
  bool has_robot_ = false;
  std::string robot_ = "";
  mc_control::CompletionCriteria crit_;
};

} // namespace fsm

} // namespace mc_control
