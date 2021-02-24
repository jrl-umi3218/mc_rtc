/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/fsm/State.h>
#include <mc_control/generic_gripper.h>

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
 * - robot: for which robot's grippers this state applies, if ommited, it is
 *   assumed to be the main robot in the controller
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
 *  "robot": "jvrc1",
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

  void teardown(Controller &) override;

protected:
  mc_rtc::Configuration config_;
  std::vector<mc_control::GripperRef> grippers_;
  bool keepSafetyConfig_ = false; /// When true, keep the gripper safety configuration after the state is destroyed
};

} // namespace fsm

} // namespace mc_control
