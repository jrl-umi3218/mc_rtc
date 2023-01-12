/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/MCController.h>

#include <mc_solver/TVMQPSolver.h>

namespace mc_control
{

/** An MCController that uses the TVM backend
 *
 * This is simply an helper class to write a TVM-only controller, the key difference with \ref MCController are:
 * - the backend is always set to Backend::TVM
 * - solver() returns a \ref mc_solver::TVMQPSolver
 */
struct MC_CONTROL_DLLAPI TVMController
: public details::BackendSpecificController<MCController::Backend::TVM, mc_solver::TVMQPSolver>
{
};

} // namespace mc_control
