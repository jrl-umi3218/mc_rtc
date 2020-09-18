/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/fsm/State.h>

namespace mc_control
{

namespace fsm
{

/** Implements a state that brings the robot posture to halfsitting stance
 *
 * Configuration options:
 * - stiffness Stiffness of the posture task
 * - completion Threshold on posture eval after which the state is considered
 *   completed
 *
 *   After state completion the PostureTask stiffness will be restored to its original
 *   value
 */
struct MC_CONTROL_FSM_STATE_DLLAPI HalfSittingState : State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(Controller &) override;

  bool run(Controller &) override;

  void teardown(Controller &) override {}

protected:
  bool has_robot_ = false;
  std::string robot_ = "";
  bool has_stiffness_ = false;
  double stiffness_ = 1;
  double eval_threshold_ = 0.01;
  bool has_eval_ = false;
  double default_stiffness_ = 1;
};

} // namespace fsm

} // namespace mc_control
