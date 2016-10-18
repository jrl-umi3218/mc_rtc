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

  /*! \brief Setup an active joints selector
   *
   * This function setups a tasks::qp;:JointsSelector for the task. Only the
   * provided joints will be used to solve the task.
   *
   * \note Calling this method or the related selectUnactiveJoints should reset
   * the current joints' selection
   *
   * \param solver Solver where the task is involved
   *
   * \param activeJointsName Active joints in the task
   *
   */
  virtual void selectActiveJoints(mc_solver::QPSolver & solver,
                                  const std::vector<std::string> & activeJointsName) = 0;

  /*! \brief Setup an unactive joints selector
   *
   * This function setups a tasks::qp;:JointsSelector for the task. All joints
   * will be used to realize the task except those provided here.
   *
   * \note Calling this method or the related selectActiveJoints should reset
   * the current joints' selection
   *
   * \param solver Solver where the task is involved
   *
   * \param unactiveJointsName Active joints in the task
   *
   */
  virtual void selectUnactiveJoints(mc_solver::QPSolver & solver,
                                    const std::vector<std::string> & unactiveJointsName) = 0;

  /*! \brief Reset active joints selection
   *
   * \param solver Solver where the task is involved
   *
   */
  virtual void resetJointsSelector(mc_solver::QPSolver & solver) = 0;

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
