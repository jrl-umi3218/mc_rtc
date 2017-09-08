#pragma once

#include <cmath>
#include <mc_solver/QPSolver.h>

#include <mc_rtc/log/Logger.h>

#include <mc_tasks/api.h>

#include <mc_rtc/Configuration.h>

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
friend struct mc_solver::QPSolver;
public:
  /** Set a name for the task
   *
   * This name will be used to identify the task in logs, GUI...
   *
   * The name should be set before being added to the solver.
   *
   */
  void name(const std::string & name) { name_ = name; }

  /** Get the name of the task */
  const std::string & name() const { return name_; }

  /*! \brief Reset the task */
  virtual void reset() = 0;

  /*! \brief Set the task's dimension weight vector
   *
   * It is the caller responsibility to ensure the dimensionality fits the
   * underlying tasks' error function
   *
   * \param dimW The new tasks's dimension weight vector
   *
   */
  virtual void dimWeight(const Eigen::VectorXd & dimW) = 0;

  /*! \brief Get the current task's dim weight vector */
  virtual Eigen::VectorXd dimWeight() const = 0;

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

  /*! \brief Load parameters from a Configuration object */
  virtual void load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config);
protected:
  /*! \brief Add the task to a solver
   *
   * \param solver Solver where to add the task
   *
   */
  virtual void addToSolver(mc_solver::QPSolver & solver) = 0;

  /*! Helper function when using another MetaTask inside a MetaTask as adding
   * the task through the solver interface has important side-effects */
  static inline void addToSolver(MetaTask & t, mc_solver::QPSolver & solver)
  {
    t.addToSolver(solver);
  }

  /*! \brief Remove the task from a solver
   *
   * \param solver Solver from which to remove the task
   *
   */
  virtual void removeFromSolver(mc_solver::QPSolver & solver) = 0;

  /*! Helper function when using another MetaTask inside a MetaTask as removing
   * the task through the solver interface has important side-effects */
  static inline void removeFromSolver(MetaTask & t, mc_solver::QPSolver & solver)
  {
    t.removeFromSolver(solver);
  }

  /*! \brief Update the task
   *
   * This function (usually) has to be called at every iteration of the solver
   * once it has been added. It should update the state of the task.
   *
   */
  virtual void update() = 0;

  /*! Helper function when using another MetaTask inside a MetaTask */
  static inline void update(MetaTask & t)
  {
    t.update();
  }

  /** Add entries to the logger
   *
   * This will be called by the solver if it holds a valid logger instance when
   * the task is added.
   *
   * The default implementation adds nothing to the log.
   */

  virtual void addToLogger(mc_rtc::Logger &) {}

  /** Remove entries from the logger
   *
   * This will be called by the solver when the task is removed.
   *
   * The default implementation removes nothing from the log.
   */
  virtual void removeFromLogger(mc_rtc::Logger &) {}

  std::string name_;
};

using MetaTaskPtr = std::shared_ptr<MetaTask>;

}
