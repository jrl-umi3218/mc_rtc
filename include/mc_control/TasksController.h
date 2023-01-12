/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/MCController.h>

#include <mc_solver/TasksQPSolver.h>

namespace mc_control
{

/** An MCController that uses the Tasks backend
 *
 * This is simply an helper class to write a Tasks-only controller, the key difference with \ref MCController are:
 * - the backend is always set to Backend::Tasks
 * - solver() returns a \ref mc_solver::TasksQPSolver
 */
struct MC_CONTROL_DLLAPI TasksController
: public details::BackendSpecificController<MCController::Backend::Tasks, mc_solver::TasksQPSolver>
{
};

} // namespace mc_control
