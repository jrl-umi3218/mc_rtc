/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/fsm/State.h>

namespace mc_control
{

namespace fsm
{

/** Implements a simple Message state to display debug information in the FSM
 *
 * This states does nothing for a while then exits with "OK"
 *
 * Configuration options:
 * - type: one of [info, error, warning, success] corresponding to the LOG_* macros
 * - message: the string to display
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
