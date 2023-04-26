/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/fsm/State.h>

namespace mc_control
{

namespace fsm
{

/** Implements a state that switches to a given controller
 *
 * This state outputs "OK" if the switch succeeds but the next state only
 * starts when you switch back to the controller that initiated the switch.
 *
 * It outputs "Failed" if the switch fails (switching to an non-enabled
 * controller).
 */
struct MC_CONTROL_FSM_STATE_DLLAPI EnableControllerState : State
{
  void start(Controller &) override;

  bool run(Controller &) override { return true; }

  void teardown(Controller &) override {}
};

} // namespace fsm

} // namespace mc_control
