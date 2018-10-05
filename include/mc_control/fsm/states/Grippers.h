#pragma once

#include <mc_control/fsm/State.h>

namespace mc_control
{

namespace fsm
{

/** Controls the gripper of a robot
 *
 * This states brings the specified grippers to a specific configuration and
 * outputs "OK" once the grippers' targets are realized.
 *
 * Configuration options:
 *
 * - grippers: each entry in this object is considered as a gripper, for each
 *   object, one can specify the gripper target as:
 *   - target: an array of double corresponding to the joint configuration of
 *   the gripper
 *   - opening: a single double value between 0 (closed) and 1
 *   (open)
 *
 * Example:
 *
 * The following configuration sets a joint target for "l_gripper" (0.7 for the
 * first joint in the gripper and 0.5 for the second joint in the gripper) and
 * a target opening percentage of 0.5 for "r_gripper".
 *
 * \code{.json}
 * {
 *  "grippers":
 *  {
 *    "l_gripper": { "target": [0.7, 0.5] },
 *    "r_gripper": { "opening": 0.5 }
 *  }
 * }
 * \endcode
 *
 */

struct MC_CONTROL_FSM_STATE_DLLAPI Grippers : State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(Controller &) override;

  bool run(Controller &) override;

  void teardown(Controller &) override {}

protected:
  mc_rtc::Configuration config_;
  std::vector<std::string> grippers_;
};

} // namespace fsm

} // namespace mc_control
