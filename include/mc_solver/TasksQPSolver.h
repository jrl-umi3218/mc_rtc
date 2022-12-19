/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_solver/QPSolver.h>

#include <Tasks/QPMotionConstr.h>
#include <Tasks/QPSolver.h>

namespace mc_solver
{

/** This implements the QPSolver interface for the Tasks backend
 *
 * This solver can accepts tasks and constraints from mc_rtc and Tasks types
 */
struct MC_SOLVER_DLLAPI TasksQPSolver final : public QPSolver
{
  TasksQPSolver(mc_rbdyn::RobotsPtr robots, double timeStep) : QPSolver(robots, timeStep, QPSolver::Backend::Tasks)
  {
    positive_lambda_constraint_.addToSolver(this->robots().mbs(), solver_);
  }

  TasksQPSolver(double timeStep) : QPSolver(timeStep, QPSolver::Backend::Tasks)
  {
    positive_lambda_constraint_.addToSolver(robots().mbs(), solver_);
  }

  ~TasksQPSolver() final = default;

  using QPSolver::addTask;

  /** Add a task to the solver
   * \param task Pointer to the added task, QPSolver does not take ownership of this pointer and the caller should make
   * sure the object remains valid until it is removed from the solver
   */
  void addTask(tasks::qp::Task * task);

  /** Add a task to the solver, ownership is shared with the solver */
  inline void addTask(std::shared_ptr<tasks::qp::Task> task)
  {
    if(task)
    {
      addTask(task.get());
      shPtrTasksStorage.emplace_back(task);
    }
  }

  using QPSolver::removeTask;

  /** Remove a task from the solver
   * \param task Pointer to the removed task. The task is not deleted after being removed
   */
  void removeTask(tasks::qp::Task * task);

  void setContacts(ControllerToken, const std::vector<mc_rbdyn::Contact> & contacts) final;

  /** Add a constraint function to the solver
   * \param constraint Pointer to the ConstraintFunction. QPSolver does not take ownserhip of this pointer and the
   * caller should make sure the object remains valid until it is removed from the solver
   */
  template<typename... Fun>
  void addConstraint(tasks::qp::ConstraintFunction<Fun...> * constraint)
  {
    constraint->addToSolver(robots().mbs(), solver_);
    solver_.updateConstrSize();
    solver_.updateNrVars(robots().mbs());
  }

  /** Remove a constraint function from the solver
   * \param constraint Pointer to the constraint that will be removed. It is not destroyed afterwards
   */
  template<typename... Fun>
  void removeConstraint(tasks::qp::ConstraintFunction<Fun...> * constraint)
  {
    constraint->removeFromSolver(solver_);
    solver_.updateConstrSize();
    solver_.updateNrVars(robots().mbs());
  }

  /** Gives access to the tasks::qp::BilateralContact entity in the solver from a contact id
   * \param id The contact id of the contact
   * \return The tasks:qp::BilateralContact entity from the solver if id is valid, otherwise, the first element of the
   * pair is -1 and the reference is invalid
   */
  std::pair<int, const tasks::qp::BilateralContact &> contactById(const tasks::qp::ContactId & id) const;

  /** Gives access to a part to lambdaVec given a contact index
   * \param cIndex The index of the contact
   * \return The lambdaVec associated
   */
  Eigen::VectorXd lambdaVec(int cIndex) const;

  const sva::ForceVecd desiredContactForce(const mc_rbdyn::Contact & id) const final;

  /** Update number of variables
   *
   * This should be called when/if you add new robots into the scene after the
   * solver initialization, this is a costly operation.
   */
  void updateNrVars();

  /** Update nr vars in tasks and constraints */
  void updateNrVars(const mc_rbdyn::Robots & robots);

  /** Update constraints matrix sizes
   *
   * \note This is mainly provided to allow safe usage of raw constraint from
   * Tasks rather than those wrapped in this library, you probably do not need
   * to call this
   */
  void updateConstrSize();

  /** Returns the internal QP solver data */
  inline tasks::qp::SolverData & data() noexcept
  {
    return solver_.data();
  }

  /** Returns the internal QP solver data (const) */
  inline const tasks::qp::SolverData & data() const noexcept
  {
    return solver_.data();
  }

  /** Access the internal QP solver */
  tasks::qp::QPSolver & solver() noexcept
  {
    return solver_;
  }

  /** Access the internal QP solver (const) */
  const tasks::qp::QPSolver & solver() const noexcept
  {
    return solver_;
  }

  /** Helper to get a \ref TasksQPSolver from a \ref QPSolver instance
   *
   * The caller should make sure the cast is valid by checking the QPSolver backend.
   *
   * In debug the program will abort otherwise, in release UB abounds
   */
  static inline TasksQPSolver & from_solver(QPSolver & solver) noexcept
  {
    assert(solver.backend() == QPSolver::Backend::Tasks);
    return static_cast<TasksQPSolver &>(solver);
  }

  /** Helper to get a \ref TasksQPSolver from a \ref QPSolver instance
   *
   * The caller should make sure the cast is valid by checking the QPSolver backend.
   *
   * In debug the program will abort otherwise, in release UB abounds
   */
  static inline const TasksQPSolver & from_solver(const QPSolver & solver) noexcept
  {
    assert(solver.backend() == QPSolver::Backend::Tasks);
    return static_cast<const TasksQPSolver &>(solver);
  }

  double solveTime() final;

  double solveAndBuildTime() final;

  const Eigen::VectorXd & result() const;

private:
  /** The actual solver instance */
  tasks::qp::QPSolver solver_;
  /** Positive lambda constraint */
  tasks::qp::PositiveLambda positive_lambda_constraint_;
  /** Holds unilateral contacts in the solver */
  std::vector<tasks::qp::UnilateralContact> uniContacts_;
  /** Holds bilateral contacts in the solver */
  std::vector<tasks::qp::BilateralContact> biContacts_;
  /** Run without feedback (open-loop) */
  bool runOpenLoop();
  /** Run with encoders' feedback */
  bool runJointsFeedback(bool wVelocity);
  /**
   * WARNING EXPERIMENTAL
   *
   * Runs the QP on an estimated robot state.
   *
   * Uses the real robot state (mbc.q and mbc.alpha) from realRobots() instances.
   * It is the users responsibility to ensure that the real robot instance is properly estimated
   * and filled. Typically, this will be done through the Observers pipeline.
   * For example, the following pipeline provides a suitable state:
   *
   * \code{.yaml}
   * RunObservers: [Encoder, KinematicInertial]
   * UpdateObservers: [Encoder, KinematicInertial]
   * \endcode
   *
   * @param integrateControlState If true, integration is performed over the control state, otherwise over the observed
   * state
   *
   * @return True if successful, false otherwise
   */
  bool runClosedLoop(bool integrateControlState);

  /** Feedback data */
  std::vector<std::vector<double>> prev_encoders_{};
  std::vector<std::vector<double>> encoders_alpha_{};
  std::vector<std::vector<std::vector<double>>> control_q_{};
  std::vector<std::vector<std::vector<double>>> control_alpha_{};

  /** Holds dynamics constraint currently in the solver */
  std::vector<mc_solver::DynamicsConstraint *> dynamicsConstraints_;

  bool run_impl(FeedbackType fType = FeedbackType::None) final;

  void addDynamicsConstraint(mc_solver::DynamicsConstraint * dynamics) final;

  void removeDynamicsConstraint(mc_solver::ConstraintSet * maybe_dynamics) final;
};

/** Helper to get a \ref TasksQPSolver from a \ref QPSolver instance
 *
 * The caller should make sure the cast is valid by checking the QPSolver backend.
 *
 * In debug the program will abort otherwise, in release UB abounds
 */
inline TasksQPSolver & tasks_solver(QPSolver & solver) noexcept
{
  return TasksQPSolver::from_solver(solver);
}

/* const version */
inline const TasksQPSolver & tasks_solver(const QPSolver & solver) noexcept
{
  return TasksQPSolver::from_solver(solver);
}

} // namespace mc_solver
