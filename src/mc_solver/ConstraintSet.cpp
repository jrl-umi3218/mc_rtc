/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_solver/ConstraintSet.h>

#include <mc_solver/TasksQPSolver.h>

namespace mc_solver
{

ConstraintSet::ConstraintSet() : backend_(mc_solver::QPSolver::context_backend()) {}

void ConstraintSet::addToSolver(mc_solver::QPSolver & solver)
{
  if(solver.backend() != backend_)
  {
    mc_rtc::log::error_and_throw(
        "[ConstraintSet::addToSolver] Creation backend ({}) is different from this solver backend ({})", backend_,
        solver.backend());
  }
  if(inSolver_) { return; }
  inSolver_ = true;
  addToSolverImpl(solver);
  switch(solver.backend())
  {
    case QPSolver::Backend::Tasks:
      static_cast<mc_solver::TasksQPSolver &>(solver).updateConstrSize();
      static_cast<mc_solver::TasksQPSolver &>(solver).updateNrVars(solver.robots());
      break;
    default:
      break;
  }
}

void ConstraintSet::removeFromSolver(mc_solver::QPSolver & solver)
{
  if(solver.backend() != backend_)
  {
    mc_rtc::log::error_and_throw(
        "[ConstraintSet::removeFromSolver] Creation backend ({}) is different from this solver backend ({})", backend_,
        solver.backend());
  }
  if(!inSolver_) { return; }
  inSolver_ = false;
  removeFromSolverImpl(solver);
  switch(solver.backend())
  {
    case QPSolver::Backend::Tasks:
      static_cast<mc_solver::TasksQPSolver &>(solver).updateConstrSize();
      static_cast<mc_solver::TasksQPSolver &>(solver).updateNrVars(solver.robots());
      break;
    default:
      break;
  }
}

} // namespace mc_solver
