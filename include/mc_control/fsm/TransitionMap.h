/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/fsm/StateFactory.h>
#include <mc_control/fsm/Transition.h>

#include <map>
#include <string>
#include <unordered_set>

namespace mc_control
{

namespace fsm
{

/** \class TransitionMap
 *
 * This class takes care of reading a transition map from a configuration
 * entry and makes sure it's coherent with the states provided by the
 * StateFactory.
 *
 * A transition map is formed by entries of the form ["StateName",
 * "OutputName", "NewStateName", "OptionalTriggerType"], e.g.:
 * - ["InitState", "OK", "GraspBar", "Strict"]
 * - ["GraspBar", "OK", "LiftLeftFoot", "StepByStep"]
 * - ["GraspBar", "NOK", "GraspBar", "Auto"]
 * - ["GraspBar", "DEFAULT", "GraspBar", "Auto"]
 *
 * If the "DEFAULT" output is present, this transition will be returned in case
 * no other output pattern has been matched.
 *
 * Valid values for the trigger type are:
 * - "StepByStep": only require user input if running in StepByStep mode
 *   (default)
 * - "Auto": automatic transition no matter what
 * - "Strict": require user input no matter what
 *
 */
struct MC_CONTROL_FSM_DLLAPI TransitionMap
{
  /** A (state, output) pair is the origin of a transition */
  using origin_t = std::pair<std::string, std::string>;

  /** Return a transition given a current state and its ouput
   *
   * \param state The current state
   *
   * \param output The state's output
   *
   * \returns A pair made of a bool and a Transition:
   * - If the (state, output) has a registered next state, the bool is true
   *   and Transition returns the corresponding transition.
   * - Else, if the (state, DEFAULT) has a registered next state, the bool is true and Transition returns the
   * corresponding transition
   * - Otherwise, the bool is false and the Transition has no meaning
   */
  std::pair<bool, Transition> transition(const std::string & state, const std::string & output) const;

  /** For a given state, gives all possible next states **/
  std::unordered_set<std::string> transitions(const std::string & state) const;

  /** Build the map from a Configuration
   *
   * The following entries are expected in config:
   * - init: initial state for this map (string)
   * - transitions: list of transitions as described in this documentation
   *   (array of strings' arrays)
   *
   * If init is absent, defaults to the first state in transitions
   *
   * \param factory The factory that will provie states
   *
   * \param config Holds the JSON representation of this transition map
   *
   */
  void init(const StateFactory & factory, const mc_rtc::Configuration & config);

  /** Returns the initial state value */
  const std::string & initState() const;

  /** Print the map */
  std::ostream & print(std::ostream & os) const;

private:
  std::string init_state_;
  std::map<origin_t, Transition> map_;
};

} // namespace fsm

} // namespace mc_control
