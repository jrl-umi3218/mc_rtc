/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_solver/QPSolver.h>

#include <memory>

namespace mc_solver
{

/** \class ConstraintSet
 * \brief This class is a basis to wrap Constraint functions from Tasks. The aim of such wrappers should be two-folds:
 *   - provide a safe alternative to the Tasks version wrapped
 *   - make the class easier to use with mc_* facilities
 */

struct MC_SOLVER_DLLAPI ConstraintSet
{
  /** Constructor, register the solver backend at creation time */
  ConstraintSet();

  /** This is called by \ref mc_solver::QPSolver when the constraint is added to the problem */
  void addToSolver(mc_solver::QPSolver & solver);

  /** This is called by \ref mc_solver::QPSolver when the constraint is removed from the problem */
  void removeFromSolver(mc_solver::QPSolver & solver);

  /** Virtual destructor */
  virtual ~ConstraintSet() {}

  inline bool inSolver() const noexcept { return inSolver_; }

  inline QPSolver::Backend backend() const noexcept { return backend_; }

protected:
  /** Should take care of the actual insertion into a concrete solver */
  virtual void addToSolverImpl(mc_solver::QPSolver & solver) = 0;

  /** Should take care of the actual removal from a concrete solver */
  virtual void removeFromSolverImpl(mc_solver::QPSolver & solver) = 0;

  /** QPSolver backend when the constraint is created */
  QPSolver::Backend backend_;

  /** True if the constraint is in a solver already */
  bool inSolver_ = false;

private:
  // Forbid copy of ConstraintSet objects
  ConstraintSet(const ConstraintSet &) = delete;
  ConstraintSet & operator=(const ConstraintSet &) = delete;
  // Move is ok
  ConstraintSet(ConstraintSet &&) = default;
  ConstraintSet & operator=(ConstraintSet &&) = default;
};

using ConstraintSetPtr = std::shared_ptr<ConstraintSet>;

} // namespace mc_solver
