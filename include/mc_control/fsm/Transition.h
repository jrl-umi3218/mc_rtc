/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/fsm/api.h>

#include <string>

namespace mc_control
{

namespace fsm
{

/** \class Transition
 *
 * A transition is given by the destination state and a type of transition.
 *
 * A type of transition dictates how the transition should be triggered.
 *
 */
struct MC_CONTROL_FSM_DLLAPI Transition
{
  std::string state;
  /** Type of transition */
  enum struct Type
  {
    /** Requires a trigger if stepping through (default) */
    StepByStep,
    /** Never requires a trigger */
    Auto,
    /** Always requires a trigger */
    Strict
  };
  Type type;
};

} // namespace fsm

} // namespace mc_control
