#pragma once

#include <mc_solver/api.h>

#include <Tasks/QPSolver.h>

namespace mc_solver
{

/** \class ConstraintSet
 * \brief This class is a basis to wrap Constraint functions from Tasks. The aim of such wrappers should be two-folds:
 *   - provide a safe alternative to the Tasks version wrapped
 *   - make the class easier to use with mc_* facilities
 */

struct MC_SOLVER_DLLAPI ConstraintSet
{
public:
  /** This function is called by mc_solver::QPSolver when
   * mc_solver::QPSolver::addConstraintSet is called, it is expected that the
   * implementation calls the safe variant of its wrapper constr:
    \verbatim
    <wrapped-type>::addToSolver(mbs, solver)
    \endverbatim
   * \param mbs The MultiBody vector controlled by the invoking mc_solver::QPSolver
   * \param solver The actual solver instance used by the invoking mc_solver::QPSolver
   */
  virtual void addToSolver(const std::vector<rbd::MultiBody> & mbs, tasks::qp::QPSolver & solver) = 0;


  /** This function is called by mc_solver::QPSolver when
   * mc_solver::QPSolver::removeConstraintSet is called, typically it would
   * call:
    \verbatim
    <wrapped-type>::removeFromSolver(solver)
    \endverbatim
   * \param solver The actual solver instance used by the invoking mc_solver::QPSolver
   */
  virtual void removeFromSolver(tasks::qp::QPSolver & solver) = 0;

  /** Virtual destructor
   */
  virtual ~ConstraintSet () {}
};

using ConstraintSetPtr = std::shared_ptr<ConstraintSet>;

} // namespace mc_solver
