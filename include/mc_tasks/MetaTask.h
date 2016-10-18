#ifndef _H_METATASK_H_
#define _H_METATASK_H_

#include <cmath>
#include <mc_solver/QPSolver.h>

#include <mc_tasks/api.h>

namespace mc_tasks
{

MC_TASKS_DLLAPI double extraStiffness(double error, double extraStiffness);

/*! \brief Represents a generic task
 *
 * A meta task may be composed of several tasks that work together to achieve a
 * given goal
 */
struct MC_TASKS_DLLAPI MetaTask
{
public:
  /*! \brief Add the task to a solver
   *
   * \param solver Solver where to add the task
   *
   */
  virtual void addToSolver(mc_solver::QPSolver & solver) = 0;

  /*! \brief Remove the task from a solver
   *
   * \param solver Solver from which to remove the task
   *
   */
  virtual void removeFromSolver(mc_solver::QPSolver & solver) = 0;

  /*! \brief Update the task
   *
   * This function (usually) has to be called at every iteration of the solver
   * once it has been added. It should update the state of the task.
   *
   */
  virtual void update() = 0;

  /*! \brief Returns the task error
   *
   * The vector's dimensions depend on the underlying task
   *
   */
  virtual Eigen::VectorXd eval() const = 0;

  /*! \brief Returns the task velocity
   *
   * The vector's dimensions depend on the underlying task
   *
   */
  virtual Eigen::VectorXd speed() const = 0;
};

}

#endif
