/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/Configuration.h>
#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/log/Logger.h>
#include <mc_solver/QPSolver.h>
#include <mc_solver/api.h>
#include <mc_tasks/api.h>

#include <cmath>

namespace mc_control
{
struct CompletionCriteria;
}

namespace mc_tasks
{

MC_SOLVER_DLLAPI double extraStiffness(double error, double extraStiffness);

/*! \brief Represents a generic task
 *
 * A meta task may be composed of several tasks that work together to achieve a
 * given goal
 */
struct MC_SOLVER_DLLAPI MetaTask
{
  friend struct mc_solver::QPSolver;
  friend struct mc_control::CompletionCriteria;

public:
  virtual ~MetaTask();

  /** Get the type of the task */
  const std::string & type() const
  {
    return type_;
  }

  /** Set a name for the task
   *
   * This name will be used to identify the task in logs, GUI...
   *
   * The name should be set before being added to the solver.
   *
   */
  void name(const std::string & name)
  {
    name_ = name;
  }

  /** Get the name of the task */
  inline const std::string & name() const
  {
    return name_;
  }

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
   * \param activeDofs Allow to select only part of the dofs of a joint
   *
   */
  virtual void selectActiveJoints(mc_solver::QPSolver & solver,
                                  const std::vector<std::string> & activeJointsName,
                                  const std::map<std::string, std::vector<std::array<int, 2>>> & activeDofs = {}) = 0;

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
   * \param unactiveDofs Allow to select only part of the dofs of a joint
   *
   */
  virtual void selectUnactiveJoints(
      mc_solver::QPSolver & solver,
      const std::vector<std::string> & unactiveJointsName,
      const std::map<std::string, std::vector<std::array<int, 2>>> & unactiveDofs = {}) = 0;

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
   * \param solver Solver in which the task is inserted
   */
  virtual void update(mc_solver::QPSolver & solver) = 0;

  /*! Helper function when using another MetaTask inside a MetaTask */
  static inline void update(MetaTask & t, mc_solver::QPSolver & solver)
  {
    t.update(solver);
  }

  /** Add entries to the logger
   *
   * This will be called by the solver if it holds a valid logger instance when
   * the task is added.
   *
   * The default implementation adds nothing to the log.
   */
  virtual void addToLogger(mc_rtc::Logger &) {}

  /*! Helper function to add a task to the logger when using another MetaTask
   * inside a MetaTask */
  static inline void addToLogger(MetaTask & t, mc_rtc::Logger & logger)
  {
    t.addToLogger(logger);
  }

  /** Remove entries from the logger
   *
   * This will be called by the solver when the task is removed.
   *
   * The default implementation removes nothing from the log.
   */
  virtual void removeFromLogger(mc_rtc::Logger &) {}

  /*! Helper function to remove a task from the logger when using another MetaTask
   * inside a MetaTask */
  static inline void removeFromLogger(MetaTask & t, mc_rtc::Logger & logger)
  {
    t.removeFromLogger(logger);
  }

  /** Add elements to the GUI through the helper
   *
   * This will be called by the solver when the task is added.
   *
   * The default implementation adds the type of the task under the {"Tasks",
   * name_} category.
   *
   */
  virtual void addToGUI(mc_rtc::gui::StateBuilder &);

  /*! Helper function to add a task to the gui when using another MetaTask
   * inside a MetaTask */
  static inline void addToGUI(MetaTask & t, mc_rtc::gui::StateBuilder & gui)
  {
    t.addToGUI(gui);
  }

  /** Remove elements from the GUI through the helper
   *
   * This will be called by the solver when the task is removed.
   *
   * The default implementation removes the {"Tasks", name_} category.
   *
   */
  virtual void removeFromGUI(mc_rtc::gui::StateBuilder &);

  /*! Helper function to remove a task from the gui when using another MetaTask
   * inside a MetaTask */
  static inline void removeFromGUI(MetaTask & t, mc_rtc::gui::StateBuilder & gui)
  {
    t.removeFromGUI(gui);
  }

  /** Add additional completion criterias to mc_control::CompletionCriteria
   * object
   *
   * Based on the input data, this should return a function that operates on
   * the task and return true and complete the output if the criteria has been
   * reached. In this function parameter, the output string should be completed
   * by the function and not overwritten. The task passed to this function is
   * *always* of the MetaTask type you're implementing and thus can be safely
   * casted.
   *
   * The default implementation is a function that returns true whatever
   * happens.
   *
   * \param dt Timestep of the completion criteria
   *
   * \param config Configuration for the CompletionCriteria
   *
   */
  virtual std::function<bool(const mc_tasks::MetaTask & task, std::string &)> buildCompletionCriteria(
      double dt,
      const mc_rtc::Configuration & config) const;

  std::string type_;
  std::string name_;
};

using MetaTaskPtr = std::shared_ptr<MetaTask>;

} // namespace mc_tasks
