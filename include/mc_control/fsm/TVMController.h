/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/fsm/Controller.h>

#include <mc_solver/TVMQPSolver.h>

namespace mc_control::fsm
{

/** An FSM Controller that uses the TVM backend */
struct MC_CONTROL_FSM_DLLAPI TVMController
: public details::BackendSpecificController<MCController::Backend::TVM, mc_solver::TVMQPSolver>
{
};

} // namespace mc_control::fsm
