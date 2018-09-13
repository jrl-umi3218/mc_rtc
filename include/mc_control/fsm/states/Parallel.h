#pragma once

#include <mc_control/fsm/State.h>

namespace mc_control
{

namespace fsm
{

/** Implements parallel states
 *
 * This states plays multiple states at once. Strictly speaking those states
 * are not played in parallel but rather sequentially.
 *
 * If this state plays {state_1, ..., state_N}. The state is completed when all
 * state_i::run() function returns true and its output is state_N output.
 *
 * Configuration entries:
 *
 * - states: list of states run by this state
 * - configs: for each state in states, configs(state) is used to further
 *   configure state
 *
 */

struct MC_CONTROL_FSM_STATE_DLLAPI ParallelState : State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(Controller&) override;

  bool run(Controller&) override;

  void stop(Controller&) override;

  void teardown(Controller&) override;

  bool read_msg(std::string & msg) override;

  bool read_write_msg(std::string & msg, std::string & out) override;
protected:
  mc_rtc::Configuration config_;
  std::vector<StatePtr> states_;
};

} // namespace fsm

} // namespace mc_control
