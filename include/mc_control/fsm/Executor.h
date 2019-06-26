/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/api.h>
#include <mc_control/fsm/State.h>
#include <mc_control/fsm/StateFactory.h>
#include <mc_control/fsm/TransitionMap.h>

#include <memory>

namespace mc_control
{

namespace fsm
{

struct Controller;

/** \class Executor takes care of executing an FSM
 *
 * The executor can works in two ways:
 *
 * - managed: handles transitions through an external trigger
 *
 * - self-managed: handles transitions thanks to a TransitionMap
 *
 */
struct MC_CONTROL_FSM_DLLAPI Executor
{
  /** Initialize the executor
   *
   * \param ctl Controller using this executor
   *
   * \param config Configuration of the executor
   *
   * \param name Name of the executor, empty for the main executor
   *
   * \param category Categry under which the executor will appear in the GUI,
   * defaults to {"FSM"} for the main executor and {"FSM", name} for other
   * executors
   *
   */
  void init(Controller & ctl,
            const mc_rtc::Configuration & config,
            const std::string & name = "",
            const std::vector<std::string> & category = {});

  /** Run the current state
   *
   * \param ctl Controller using this executor
   *
   * \param keep_state If true, keep the state when it finishes its run
   *
   * \returns True if the execution is complete
   *
   */
  bool run(Controller & ctl, bool keep_state);

  /** Stop the current stat eif necessary
   *
   * \param ctl Controller using this executor
   *
   */
  void stop(Controller & ctl);

  /** Teardown the current state if necessary
   *
   * \param ctl Controller using this executor
   *
   */
  void teardown(Controller & ctl);

  /** Trigger an interruption */
  inline void interrupt()
  {
    interrupt_triggered_ = true;
  }

  /** Returns true if the state is active */
  inline bool running() const
  {
    return state_ != nullptr;
  }

  /** Returns true if the executor is ready for next transition */
  inline bool ready() const
  {
    return ready_;
  }

  /** Returns true if the executor reached the end of transition map */
  inline bool complete() const
  {
    return complete_;
  }

  /** Resume execution to a given state
   *
   * Interrupt current state execution if needed
   */
  bool resume(const std::string & state);

  /** Trigger next state
   *
   * \returns False if the FSM is not ready for next state
   */
  bool next();

  /** Returns the current state's name */
  const std::string & state() const
  {
    return curr_state_;
  }

  /** Returns the latest state's output */
  const std::string & output() const
  {
    return state_output_;
  }

  /** Returns the next state's name */
  const std::string & next_state() const
  {
    return next_state_;
  }

  /** Pass message to current state (read-only) */
  bool read_msg(std::string & msg);

  /** Pass message to current state (read-write) */
  bool read_write_msg(std::string & msg, std::string & out);

private:
  /** Configuration passed at construction, can hold specific states' configuration */
  mc_rtc::Configuration config_;
  /** Name of the executor */
  std::string name_;
  /** Category under which the executor controls should appear in the GUI */
  std::vector<std::string> category_;
  /** True if managed */
  bool managed_ = false;
  /** If true and not managed, waits for trigger before transitions */
  bool step_by_step_ = true;

  /** Transition map, empty if managed */
  TransitionMap transition_map_;

  /** Current state */
  StatePtr state_ = nullptr;
  /** Curent state (name) */
  std::string curr_state_ = "";
  /** State output */
  std::string state_output_ = "";

  /** If true, the state has been interrupted */
  bool interrupt_triggered_ = false;
  /** If true, executor is ready for next transition */
  bool ready_ = true;
  /** If true, transition has been triggered */
  bool transition_triggered_ = false;
  /** If true, no more state can be executed by this FSM */
  bool complete_ = false;
  /** Name of the next state */
  std::string next_state_ = "";

private:
  /** Complete execution */
  bool complete(Controller & ctl, bool keep_state);

  /** Setup next state */
  void next(Controller & ctl);
};

} // namespace fsm

} // namespace mc_control
