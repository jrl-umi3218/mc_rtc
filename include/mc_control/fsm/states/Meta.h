/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/fsm/Executor.h>

namespace mc_control
{

namespace fsm
{

/** Implements a "meta" state
 *
 * This states plays its own FSM.
 *
 * Configuration entries:
 *
 * - Managed: if true, does not handle transitions
 * - StepByStep: same as FSM for the internal FSM (default: false)
 * - transitions: a transition map, similiar to the FSM controller (required if Managed is false)
 * - category: an arrray of strings, dictates where the executor adds elements into the GUI
 * - configs: can contain additional configuration for the states in the FSM
 *
 */
struct MC_CONTROL_FSM_STATE_DLLAPI MetaState : State
{
  void start(Controller &) override;

  bool run(Controller &) override;

  void stop(Controller &) override;

  void teardown(Controller &) override;

  bool read_msg(std::string & msg) override;

  std::vector<std::vector<std::string>> transitions() const;

  std::map<std::string, mc_rtc::Configuration> configs() const;

protected:
  Executor executor_;
};

} // namespace fsm

} // namespace mc_control
