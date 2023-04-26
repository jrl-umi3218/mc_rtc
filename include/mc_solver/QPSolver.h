/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_solver/api.h>

#include <mc_control/api.h>
#include <mc_rbdyn/Contact.h>
#include <mc_rbdyn/Robots.h>

#include <mc_rtc/pragma.h>

#include <memory>

namespace mc_tasks
{

struct MetaTask;

} // namespace mc_tasks

namespace mc_rtc
{

struct Logger;

namespace gui
{

struct StateBuilder;

} // namespace gui

} // namespace mc_rtc

namespace mc_control
{

struct MCController;

} // namespace mc_control

namespace mc_solver
{

// Work around GCC bug see: https://gcc.gnu.org/bugzilla/show_bug.cgi?id=43407
MC_RTC_diagnostic_push
MC_RTC_diagnostic_ignored(GCC, "-Wattributes")

/** Describe the type of feedback used to control the robot */
enum class MC_SOLVER_DLLAPI FeedbackType
{
  /** No feedback, i.e. open-loop control */
  None,
  /** Synonyn for None */
  OpenLoop = None,
  /** Use encoder values for actuated joints */
  Joints,
  /** Joints + encoder velocity obtained from numerical differentiation */
  JointsWVelocity,
  /** Run in closed loop w.r.t realRobots using the observation pipeline and integrate over the control state of the
     system */
  ObservedRobots,
  /** Synonym for ObservedRobots */
  ClosedLoop = ObservedRobots,
  /** Run in closed loop w.r.t realRobots using the observation pipeline and integrate over the real state of the system
   */
  ClosedLoopIntegrateReal
};

MC_RTC_diagnostic_pop

struct ConstraintSet;
struct DynamicsConstraint;
struct TasksQPSolver;

/** \class QPSolver
 *
 * Abstract base class for wrapping a solver
 *
 * Always ensure that the solver is up-to-date
 */
struct MC_SOLVER_DLLAPI QPSolver
{
public:
  /** Known backends for QPSolver implementation, tasks and constraints in
   * mc_rtc can discriminate on this value to select the implementation */
  enum class Backend
  {
    /** Unset */
    Unset,
    /** Use Tasks library as a backend */
    Tasks,
    /** Use TVM library as a backend */
    TVM
  };

  /** This token is used to give mc_control::MCController access to some internals */
  struct MC_SOLVER_DLLAPI ControllerToken
  {
    friend struct mc_control::MCController;
    friend struct QPSolver;

  private:
    ControllerToken() = default;
  };

  /** Constructor
   * \param robots Set of robots managed by this solver
   * \param timeStep Timestep of the solver
   *
   * \note The real robots will be created by copying the provided robots
   */
  QPSolver(mc_rbdyn::RobotsPtr robots, double timeStep, Backend backend);

  /** Constructor (the solver creates its own Robots instance)
   * \param timeStep Timestep of the solver
   */
  QPSolver(double timeStep, Backend backend);

  virtual ~QPSolver() = default;

  /** Returns the backend for this solver instance */
  inline Backend backend() const noexcept { return backend_; }

  /** Returns the current context backend
   *
   * This should be set whenever constraints or tasks are about to be created and it is known they don't use the solver
   * object and thus cannot determine the correct backend
   *
   * For example:
   * - QPSolver implementations should set it on the thread it's created
   * - ConstraintSetLoader and MetaTasksLoader set it before loading objects
   * - MCGlobalController sets it before run
   * - You might need to set it if you create tasks in a thread you created
   */
  static Backend context_backend();

  /** Set the current context backend */
  static void context_backend(Backend backend);

  /** Add a constraint set
   * \param cs Constraint set added to the solver
   */
  void addConstraintSet(ConstraintSet & cs);

  /** Add a constraint set
   *
   * \param cs Constraint set added to the solver
   */
  template<typename T>
  inline void addConstraintSet(const std::unique_ptr<T> & ptr)
  {
    addConstraintSet(*ptr);
  }

  /** Remove a constraint set
   * \param cs Constrain set removed from the solver
   */
  void removeConstraintSet(ConstraintSet & cs);

  /** Remove a constraint set
   *
   * \param cs Constraint set removed from the solver
   */
  template<typename T>
  inline void removeConstraintSet(const std::unique_ptr<T> & ptr)
  {
    removeConstraintSet(*ptr);
  }

  /** Add a task to the solver
   *
   * Adding the same task multiple times has no effect.
   *
   * \param task Pointer to an mc_tasks::MetaTask, QPSolver does not take
   * ownership of this pointer. The MetaTask update function will be
   * automatically called before the optimization is solved.
   *
   */
  void addTask(mc_tasks::MetaTask * task);

  /** Add a task to the solver, ownership is shared with the solver */
  inline void addTask(std::shared_ptr<mc_tasks::MetaTask> task)
  {
    if(task)
    {
      addTask(task.get());
      shPtrTasksStorage.emplace_back(task);
    }
  }

  /** Remove a task from the solver
   *
   * Removing a task that is not in the solver has no effect.
   *
   * \param task Pointer to an mc_tasks::MetaTask. The task will not be
   * updated anymore and memory should be released by the task's owner.
   *
   */
  void removeTask(mc_tasks::MetaTask * task);

  /** Remove a task from the solver which was shared with the solver */
  inline void removeTask(std::shared_ptr<mc_tasks::MetaTask> task)
  {
    if(task) { removeTask(task.get()); }
  }

  /** Reset all contacts in the solver and use the new set of contacts provided
   * \item contact Set of mc_rbdyn::Contact
   */
  void setContacts(const std::vector<mc_rbdyn::Contact> & contacts = {});

  /* Called by the owning controller to actually set the contacts or internally by QPSolver when it has no owning
   * controller */
  virtual void setContacts(ControllerToken, const std::vector<mc_rbdyn::Contact> & contacts) = 0;

  /** Returns the current set of contacts */
  const std::vector<mc_rbdyn::Contact> & contacts() const;

  /** Returns the MetaTasks currently in the solver */
  const std::vector<mc_tasks::MetaTask *> & tasks() const;

  /** Desired resultant of contact force in robot surface frame
   * \param contact Contact for which the force is desired.
   * This contact must be one of the active contacts in the solver.
   * \return Contact force in robot surface frame
   */
  virtual const sva::ForceVecd desiredContactForce(const mc_rbdyn::Contact & id) const = 0;

  /** Run one iteration of the QP.
   *
   * If succesful, will update the robots' configurations
   *
   * \param fType Type of feedback used to close the loop on sensory information
   *
   * \return True if successful, false otherwise.
   */
  bool run(FeedbackType fType = FeedbackType::None);

  /** Gives access to the main robot in the solver */
  const mc_rbdyn::Robot & robot() const;
  /** Gives access to the main robot in the solver */
  mc_rbdyn::Robot & robot();

  /** Gives access to the robot with the given index in the solver */
  mc_rbdyn::Robot & robot(unsigned int idx);
  /** Gives access to the robot with the given index in the solver */
  const mc_rbdyn::Robot & robot(unsigned int idx) const;

  /** Gives access to the environment robot in the solver (see mc_rbdyn::Robots) */
  const mc_rbdyn::Robot & env() const;
  /** Gives access to the environment robot in the solver (see mc_rbdyn::Robots) */
  mc_rbdyn::Robot & env();

  /** Gives access to the robots controlled by this solver */
  const mc_rbdyn::Robots & robots() const;
  /** Gives access to the robots controlled by this solver */
  mc_rbdyn::Robots & robots();

  /** Gives access to the real robots used by this solver */
  const mc_rbdyn::Robots & realRobots() const;
  /** Gives access to the real robots used by this solver */
  mc_rbdyn::Robots & realRobots();

  /** Returns the timestep of the solver
   * \return The timestep of the solver
   */
  double dt() const;

  /** Returns the solving time in ms */
  virtual double solveTime() = 0;

  /** Returns the building and solving time in ms */
  virtual double solveAndBuildTime() = 0;

  /** Set the logger for this solver instance */
  void logger(std::shared_ptr<mc_rtc::Logger> logger);
  /** Access to the logger instance */
  std::shared_ptr<mc_rtc::Logger> logger() const;

  /** Set the GUI helper for this solver instance */
  void gui(std::shared_ptr<mc_rtc::gui::StateBuilder> gui);
  /** Access to the gui instance */
  std::shared_ptr<mc_rtc::gui::StateBuilder> gui() const;

  /** Set the controller that is owning this QPSolver instance */
  inline void controller(mc_control::MCController * ctl) noexcept { controller_ = ctl; }
  /** Returns the controller owning this instance (if any) (const) */
  inline const mc_control::MCController * controller() const noexcept { return controller_; }
  /** Returns the controller owning this instance (if any) */
  inline mc_control::MCController * controller() noexcept { return controller_; }

protected:
  Backend backend_;
  mc_rbdyn::RobotsPtr robots_p;
  mc_rbdyn::RobotsPtr realRobots_p;
  double timeStep;

  /** Holds mc_rbdyn::Contact in the solver */
  std::vector<mc_rbdyn::Contact> contacts_;

  /** Holds MetaTask currently in the solver */
  std::vector<mc_tasks::MetaTask *> metaTasks_;

  /** Storage for shared_pointer on tasks */
  std::vector<std::shared_ptr<void>> shPtrTasksStorage;

  /** Pointer to the Logger */
  std::shared_ptr<mc_rtc::Logger> logger_ = nullptr;

  /** Pointer to the GUI helper */
  std::shared_ptr<mc_rtc::gui::StateBuilder> gui_ = nullptr;

  void addTaskToGUI(mc_tasks::MetaTask * task);

  /** Can be nullptr if this not associated to any controller */
  mc_control::MCController * controller_ = nullptr;

  /** Should run the control prroblem and update the control robot accordingly */
  virtual bool run_impl(FeedbackType fType = FeedbackType::None) = 0;

  /** This is called when a dynamics constraint is added to the solver */
  virtual void addDynamicsConstraint(mc_solver::DynamicsConstraint * dynamics) = 0;

  /** This is called anytime a constraint is removed, the passed constraint is not always a dynamics constraint */
  virtual void removeDynamicsConstraint(mc_solver::ConstraintSet * maybe_dynamics) = 0;
};

} // namespace mc_solver

namespace fmt
{

template<>
struct formatter<mc_solver::QPSolver::Backend> : public formatter<string_view>
{
  template<typename FormatContext>
  auto format(const mc_solver::QPSolver::Backend & backend, FormatContext & ctx) -> decltype(ctx.out())
  {
    using Backend = mc_solver::QPSolver::Backend;
    switch(backend)
    {
      case Backend::Tasks:
        return formatter<string_view>::format("Tasks", ctx);
      case Backend::TVM:
        return formatter<string_view>::format("TVM", ctx);
      case Backend::Unset:
        return formatter<string_view>::format("Unset", ctx);
      default:
        return formatter<string_view>::format("UNEXPECTED", ctx);
    }
  }
};

} // namespace fmt
