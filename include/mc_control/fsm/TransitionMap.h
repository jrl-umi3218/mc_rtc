#pragma once

#include <mc_control/fsm/StateFactory.h>
#include <mc_control/fsm/Transition.h>

#include <map>
#include <string>

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
 *
 * Valid values for the trigger type are:
 * - "StepByStep": only require user input if running in StepByStep mode
 *   (default)
 * - "Auto": automatic transition no matter what
 * - "Strict": require user input no matter what
 *
 */
struct MC_CONTROL_DLLAPI TransitionMap
{
  /** A (state, output) pair is the origin of a transition */
  using origin_t = std::pair<std::string, std::string>;

  /** Return a transition given a current state and its ouput
   *
   * \param state The current state
   *
   * \param output The state's output
   *
   * \returns A pair made of a bool and a Transition. If the (state,
   * output) has a registered next state, the bool is true and Transition
   * returns the corresponding transition. Otherwise, the bool is false
   * and the Transition has no meaning.
   *
   */
   std::pair<bool, Transition> transition(const std::string & state,
                                          const std::string & output) const;

  /** Build the map from a Configuration */
  void init(const StateFactory & factory,
            const mc_rtc::Configuration & config);

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
