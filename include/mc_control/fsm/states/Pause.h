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
struct MC_CONTROL_FSM_STATE_DLLAPI PauseState : State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(Controller &) override;

  bool run(Controller &) override;

  void teardown(Controller &) override {}

protected:
  double duration_ = 0.0;
  unsigned int tick_;
  unsigned int goal_;
};

} // namespace fsm

} // namespace mc_control
