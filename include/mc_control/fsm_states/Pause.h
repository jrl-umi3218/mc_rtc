#pragma once

#include <mc_control/mc_fsm_state.h>

namespace mc_control
{

/** Implements a pause state
 *
 * This states does nothing for a while then exits with "OK"
 *
 * Configuration options:
 * - duration Duration of the pause in seconds (double, defaults: 0)
 *
 */
struct PauseState : mc_control::FSMState
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(FSMController&) override;

  bool run(FSMController&) override;

  void teardown(FSMController&) override {}
protected:
  double duration_ = 0.0;
  unsigned int tick_;
  unsigned int goal_;
};

}
