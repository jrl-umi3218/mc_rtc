/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/MCController.h>
#include <mc_control/fsm/Executor.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/PostureTask.h>

#include <functional>

namespace mc_control
{

namespace fsm
{

struct Controller;

using Contact = mc_control::Contact;

} // namespace fsm

} // namespace mc_control

namespace mc_control
{

namespace fsm
{

using ContactSet = mc_control::ContactSet;

/** \class Controller
 *
 * This controller implements a finite-state machine to handle complex
 * scenarios in a generic fashion.
 *
 * The FSM can be controlled in two ways:
 *
 * - managed: in that case, an external tool triggers transitions
 *
 * - self-managed: the FSM takes care of transition, in that case a
 *   transition map must be provided.
 *
 */
struct MC_CONTROL_FSM_DLLAPI Controller : public MCController
{
  friend struct Executor;

  Controller(std::shared_ptr<mc_rbdyn::RobotModule> rm, double dt, const mc_rtc::Configuration & config);

  ~Controller() override;

  bool run() override;

  /** Can be called in derived class to run the FSM with a different feedback strategy
   *
   * \param fType Type of feedback used in the solver
   *
   */
  bool run(mc_solver::FeedbackType fType);

  void reset(const ControllerResetData & data) override;

  /** Stop the current state's execution
   *
   * The controller will switch to its idle behaviour which maintains current
   * contacts, free-flyer position/orientation and posture.
   *
   * This function is virtual to allow derived implementation to handle
   * interruptions differently.
   */
  virtual void interrupt()
  {
    executor_.interrupt();
  }

  /** Check if current state is running */
  bool running()
  {
    return executor_.running();
  }

  /** Resume the FSM execution on a new state
   *
   * This call is ignored if running() returns true
   *
   * \param state State to start
   *
   * \return True if the state was started
   *
   */
  bool resume(const std::string & state);

  /** Get the posture task associated to a robot
   *
   * Returns a nullptr if the task does not exist (i.e. robot is not
   * actuated or not loaded by the controller)
   *
   */
  std::shared_ptr<mc_tasks::PostureTask> getPostureTask(const std::string & robot);

  /** Access contact constraint */
  mc_solver::ContactConstraint & contactConstraint()
  {
    return *contact_constraint_;
  }

  /** Access the state factory */
#ifndef MC_RTC_BUILD_STATIC
  StateFactory & factory()
  {
    return factory_;
  }
#else
  static StateFactory & factory()
  {
    if(!factory_ptr_)
    {
      factory_ptr_.reset(new StateFactory({}, {}, false));
    }
    return *factory_ptr_;
  }
#endif

private:
  /** Reset all posture tasks */
  void resetPostures();

  /** Start the idle state */
  void startIdleState();

  /** Teardown the idle state */
  void teardownIdleState();

protected:
  /** Creates a posture task for each actuated robots
   * (i.e. robot.dof() - robot.joint(0).dof() > 0 ) */
  std::map<std::string, std::shared_ptr<mc_tasks::PostureTask>> posture_tasks_;
  std::map<std::string, double> saved_posture_weights_;

  /** Creates a free-flyer end-effector task for each robot with a free flyer */
  std::map<std::string, std::shared_ptr<mc_tasks::EndEffectorTask>> ff_tasks_;

  /** State factory */
#ifndef MC_RTC_BUILD_STATIC
  StateFactory factory_;
#else
  static std::unique_ptr<StateFactory> factory_ptr_;
  StateFactory & factory_;
#endif
  /** Behaviour during idle */
  bool idle_keep_state_ = false;
  /** True if not idle */
  bool running_ = false;
  /** Main executor */
  Executor executor_;
};

} // namespace fsm

} // namespace mc_control
