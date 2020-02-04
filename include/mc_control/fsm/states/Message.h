/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/fsm/State.h>

namespace mc_control
{

namespace fsm
{

/** Implements a pause state
 *
 * This states does nothing for a while then exits with "OK"
 *
 * Configuration options:
 * - duration Duration of the pause in seconds (double, defaults: 0)
 *
 */
struct MC_CONTROL_FSM_STATE_DLLAPI MessageState : State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(Controller &) override;

  bool run(Controller &) override;

  void teardown(Controller &) override {}

protected:
  std::string message_;
  std::string type_ = "info";
};

} // namespace fsm

} // namespace mc_control
