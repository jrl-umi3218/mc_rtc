/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/fsm/Controller.h>

#include <mc_solver/TasksQPSolver.h>

namespace mc_control::fsm
{

/** An FSM Controller that uses the Tasks backend */
struct MC_CONTROL_FSM_DLLAPI TasksController
: public details::BackendSpecificController<MCController::Backend::Tasks, mc_solver::TasksQPSolver>
{
  using details::BackendSpecificController<MCController::Backend::Tasks,
                                           mc_solver::TasksQPSolver>::BackendSpecificController;
};

} // namespace mc_control::fsm
