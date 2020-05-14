/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/fsm/State.h>

namespace mc_control
{

namespace fsm
{

/** Implements a simple Message state to display debug information in the FSM
 *
 * Messages can be shown in the CLI as LOG_* macro calls, or in the GUI as Label
 * elements.
 *
 * This state always outputs "OK" and is always completed.
 */
struct MC_CONTROL_FSM_STATE_DLLAPI MessageState : State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(Controller &) override;

  bool run(Controller &) override;

  void teardown(Controller &) override;

protected:
  std::string prefix_;
  std::string message_;
  std::string logType_ = "info";
  bool gui_ = false;
  std::vector<std::string> guiCategory_;
  std::string labelName_;
};

} // namespace fsm

} // namespace mc_control
