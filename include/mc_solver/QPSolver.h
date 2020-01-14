/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifndef _H_MCCONTROLQPSOLVER_H_
#define _H_MCCONTROLQPSOLVER_H_

#include <mc_rbdyn/Contact.h>
#include <mc_rbdyn/Robots.h>
#include <mc_solver/ConstraintSet.h>
#include <mc_solver/DynamicsConstraint.h>
#include <mc_solver/api.h>
#include <mc_solver/msg/QPResult.h>

#include <Tasks/QPSolver.h>

#include <memory>

namespace mc_tasks
{
struct MetaTask;
}

namespace mc_rtc
{
struct Logger;
namespace gui
{
struct StateBuilder;
}
} // namespace mc_rtc

namespace mc_solver
{

#pragma GCC diagnostic push
// Work around GCC bug see: https://gcc.gnu.org/bugzilla/show_bug.cgi?id=43407
#pragma GCC diagnostic ignored "-Wattributes"

/** Describe the type of feedback used to control the robot */
enum class MC_SOLVER_DLLAPI FeedbackType
{
  /** No feedback, i.e. open-loop control */
  None,
  /** Use encoder values for actuated joints */
  Joints,
  /** Joints + encoder velocity obtained from numerical differentiation */
  JointsWVelocity
};

#pragma GCC diagnostic pop

/** \class QPSolver
 *
 * Wraps a tasks::qp::QPSolver instance
 *
 * Always ensure that the solver is up-to-date
 */

struct MC_SOLVER_DLLAPI QPSolver
{
public:
  /** Constructor
   * \param robot Set of robots managed by this solver
   * \param timeStep Timestep of the solver
   */
  QPSolver(std::shared_ptr<mc_rbdyn::Robots> robots, double timeStep);

  /** Constructor (the solver creates its own Robots instance)
   * \param timeStep Timestep of the solver
   */
  QPSolver(double timeStep);

  /** Add a constraint set
   * \param cs Constraint set added to the solver
   */
  void addConstraintSet(ConstraintSet & cs);

  /** Remove a constraint set
   * \param cs Constrain set removed from the solver
   */
  void removeConstraintSet(ConstraintSet & cs);

  /** Add a task to the solver
   * \param task Pointer to the added task, QPSolver does not take ownership of this pointer and the caller should make
   * sure the object remains valid until it is removed from the solver
   */
  void addTask(tasks::qp::Task * task);

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

  /** Add a task to the solver
   *
   * Simple wrapper to add a shared_ptr
   *
   * \param task Shared-pointer to a task T that is derived from
   * mc_tasks::MetaTask
   *
   */
  template<typename T>
  inline void addTask(std::shared_ptr<T> task)
  {
    static_assert(std::is_base_of<mc_tasks::MetaTask, T>::value || std::is_base_of<tasks::qp::Task, T>::value,
                  "You are trying to add a task that is neither a tasks::qp::Task or an mc_tasks::MetaTask");
    if(task)
    {
      addTask(task.get());
      shPtrTasksStorage.emplace_back(task);
    }
  }

  /** Remove a task from the solver
   * \param task Pointer to the removed task. The task is not deleted after being removed
   */
  void removeTask(tasks::qp::Task * task);

  /** Remove a task from the solver
   *
   * Removing a task that is not in the solver has no effect.
   *
   * \param task Pointer to an mc_tasks::MetaTask. The task will not be
   * updated anymore and memory should be released by the task's owner.
   *
   */
  void removeTask(mc_tasks::MetaTask * task);

  /** Remove a task from the solver
   *
   * Simple wrapper to remove a shared_ptr
   *
   * \param task Shared-pointer to a task T that is derived from
   * mc_tasks::MetaTask
   *
   */
  template<typename T>
  inline void removeTask(std::shared_ptr<T> task)
  {
    static_assert(std::is_base_of<mc_tasks::MetaTask, T>::value || std::is_base_of<tasks::qp::Task, T>::value,
                  "You are trying to add a task that is neither a tasks::qp::Task or an mc_tasks::MetaTask");
    if(task)
    {
      removeTask(task.get());
    }
  }

  /** Add a constraint function from the solver
   * \param constraint Pointer to the ConstraintFunction. QPSolver does not take ownserhip of this pointer and the
   * caller should make sure the object remains valid until it is removed from the solver
   */
  template<typename... Fun>
  void addConstraint(tasks::qp::ConstraintFunction<Fun...> * constraint)
  {
    constraint->addToSolver(robots().mbs(), solver);
  }

  /** Remove a constraint function from the solver
   * \param constraint Pointer to the constraint that will be removed. It is not destroyed afterwards
   */
  template<typename... Fun>
  void removeConstraint(tasks::qp::ConstraintFunction<Fun...> * constraint)
  {
    constraint->removeFromSolver(solver);
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

  /** Reset all contacts in the solver and use the new set of contacts provided
   * \item contact Set of mc_rbdyn::Contact
   */
  void setContacts(const std::vector<mc_rbdyn::Contact> & contacts = {});

  /** Returns the current set of contacts */
  const std::vector<mc_rbdyn::Contact> & contacts() const;

  /** Returns the MetaTasks currently in the solver */
  const std::vector<mc_tasks::MetaTask *> & tasks() const;

  /** Desired resultant of contact force in robot surface frame
   * \param contact Contact for which the force is desired.
   * This contact must be one of the active contacts in the solver.
   * \return Contact force in robot surface frame
   */
  const sva::ForceVecd desiredContactForce(const mc_rbdyn::Contact & id) const;

  /** Run one iteration of the QP.
   *
   * If succesful, will update the robots' configurations
   *
   * \param fType Type of feedback used to close the loop on sensory information
   *
   * \return True if successful, false otherwise.
   */
  bool run(FeedbackType fType = FeedbackType::None);

  /**
   * WARNING EXPERIMENTAL
   *
   * Runs the QP on an estimated robot state
   *
   * @param robot_est
   *  Estimated robot state. Both mbc().q and mbc().alpha should be defined
   *
   * @return True if successful, false otherwise
   */
  bool runClosedLoop(std::shared_ptr<mc_rbdyn::Robots> robot_est);

  /** Provides the result of run() for robots.robot()
   * \param curTime Unused
   */
  const QPResultMsg & send(double curTime = 0);

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

  /** Allows to set the real robots used by this solver
   * XXX could be dangerous / misleading if users set it but tasks have stored
   * it too
   */
  void realRobots(std::shared_ptr<mc_rbdyn::Robots> realRobots);
  /** Gives access to the real robots used by this solver */
  const mc_rbdyn::Robots & realRobots() const;
  /** Gives access to the real robots used by this solver */
  mc_rbdyn::Robots & realRobots();

  /** Update number of variables
   *
   * This should be called when/if you add new robots into the scene after the
   * solver initialization, this is a costly operation.
   */
  void updateNrVars();

  /** Update constraints matrix sizes
   *
   * \note This is mainly provided to allow safe usage of raw constraint from
   * Tasks rather than those wrapped in this library, you probably do not need
   * to call this
   */
  void updateConstrSize();

  /** Returns the timestep of the solver
   * \return The timestep of the solver
   */
  double dt() const;

  /** Returns the internal QP solver data
   * \return The data of the solver
   */
  tasks::qp::SolverData & data();

  /** Use the dynamics constraint to fill torque in the main robot */
  void fillTorque(const mc_solver::DynamicsConstraint & dynamicsConstraint);

  boost::timer::cpu_times solveTime();

  boost::timer::cpu_times solveAndBuildTime();

  /** Return the solvers result vector.
   * \return The solvers result vector.
   */
  const Eigen::VectorXd & result() const;

  /** Set the logger for this solver instance */
  void logger(std::shared_ptr<mc_rtc::Logger> logger);
  /** Access to the logger instance */
  std::shared_ptr<mc_rtc::Logger> logger() const;

  /** Set the GUI helper for this solver instance */
  void gui(std::shared_ptr<mc_rtc::gui::StateBuilder> gui);
  /** Access to the gui instance */
  std::shared_ptr<mc_rtc::gui::StateBuilder> gui() const;

private:
  std::shared_ptr<mc_rbdyn::Robots> robots_p;
  std::shared_ptr<mc_rbdyn::Robots> realRobots_p;
  double timeStep;

  /** Holds mc_rbdyn::Contact in the solver */
  std::vector<mc_rbdyn::Contact> contacts_;
  /** Holds unilateral contacts in the solver */
  std::vector<tasks::qp::UnilateralContact> uniContacts;
  /** Holds bilateral contacts in the solver */
  std::vector<tasks::qp::BilateralContact> biContacts;

  /** Holds MetaTask currently in the solver */
  std::vector<mc_tasks::MetaTask *> metaTasks_;

private:
  /** The actual solver instance */
  tasks::qp::QPSolver solver;
  /** Latest result */
  QPResultMsg qpRes;

  std::vector<std::shared_ptr<void>> shPtrTasksStorage;

  /** Update qpRes from the latest run() */
  void __fillResult();

  /** Pointer to the Logger */
  std::shared_ptr<mc_rtc::Logger> logger_ = nullptr;

  /** Pointer to the GUI helper */
  std::shared_ptr<mc_rtc::gui::StateBuilder> gui_ = nullptr;

  void addTaskToGUI(mc_tasks::MetaTask * task);

  /** Run without feedback (open-loop) */
  bool runOpenLoop();

  /** Run with encoders' feedback */
  bool runJointsFeedback(bool wVelocity);

  /** Feedback data */
  std::vector<std::vector<double>> prev_encoders_{};
  std::vector<std::vector<double>> encoders_alpha_{};
  std::vector<std::vector<std::vector<double>>> control_q_{};
  std::vector<std::vector<std::vector<double>>> control_alpha_{};

public:
  /** \deprecated{Default constructor, not made for general usage} */
  QPSolver() {}
};

} // namespace mc_solver

#endif
