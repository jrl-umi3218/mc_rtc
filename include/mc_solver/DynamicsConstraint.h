/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_solver/KinematicsConstraint.h>

#include <mc_tvm/DynamicFunction.h>

#include <Tasks/QPMotionConstr.h>

namespace mc_solver
{

/** \class DynamicsConstraint
 * Holds dynamics constraints (equation of motion) for a robot
 */

struct MC_SOLVER_DLLAPI DynamicsConstraint : public KinematicsConstraint
{
public:
  /** Constructor
   * Builds a regular joint limits constraint and a motion constr depending on
   * the nature of the robot
   * See tasks::qp::MotionConstr for details on the latter one
   * \param robots The robots including the robot affected by this constraint
   * \param robotIndex The index of the robot affected by this constraint
   * \param timeStep Solver timestep
   * \param infTorque If true, ignore the torque limits set in the robot model
   * \param compensateExtTorques If true, compensates external disturbances using a feedworward torque signal. The
   * constraint will search for the compensation value in robot by calling `compensationTorques()` method, if not an
   * estimation of external torques acting on the robot will be used by calling `externalTorques()` method.
   */
  DynamicsConstraint(const mc_rbdyn::Robots & robots,
                     unsigned int robotIndex,
                     double timeStep,
                     bool infTorque = false,
                     bool compensateExtTorques = false);

  /** Constructor
   * Builds a damped joint limits constraint and a motion constr depending on
   * the nature of the robot
   * See tasks::qp::MotionConstr for details on the latter one
   * \param robots The robots including the robot affected by this constraint
   * \param robotIndex The index of the robot affected by this constraint
   * \param timeStep Solver timestep
   * \param damper Value of the damper {interaction distance, safety distance,
   * offset}
   * \param velocityPercent Maximum joint velocity percentage, 0.5 is advised
   * \param infTorque If true, ignore the torque limits set in the robot model
   * \param compensateExtTorques If true, compensates external disturbances using a feedworward torque signal. The
   * constraint will search for the compensation value in robot by calling `compensationTorques()` method, if not an
   * estimation of external torques acting on the robot will be used by calling `externalTorques()` method.
   */
  DynamicsConstraint(const mc_rbdyn::Robots & robots,
                     unsigned int robotIndex,
                     double timeStep,
                     const std::array<double, 3> & damper,
                     double velocityPercent = 1.0,
                     bool infTorque = false,
                     bool compensateExtTorques = false);

  /** \brief Update the constraint
   *
   * This is called at every iteration of the controller once the constraint has been added to a solver
   *
   * \param solver Solver in which the constraint has been inserted
   */
  void update(QPSolver & solver) override;

  /** Returns the tasks::qp::MotionConstr
   *
   * This assumes the backend was Tasks
   */
  inline tasks::qp::MotionConstr & motionConstr() noexcept
  {
    assert(backend_ == QPSolver::Backend::Tasks);
    return *static_cast<tasks::qp::MotionConstr *>(motion_constr_.get());
  }

  /** Returns the mc_tvm::DynamicFunction
   *
   * Assumes the backend was TVM
   */
  inline mc_tvm::DynamicFunction & dynamicFunction()
  {
    assert(backend_ == QPSolver::Backend::TVM);
    return *(static_cast<mc_tvm::DynamicFunctionPtr *>(motion_constr_.get())->get());
  }

  void addToSolverImpl(QPSolver & solver) override;

  void removeFromSolverImpl(QPSolver & solver) override;

  inline unsigned int robotIndex() const noexcept { return robotIndex_; }

protected:
  /** Holds the motion constraint implementation
   *
   * In Tasks backend:
   * - tasks::qp::MotionConstr or a derived constraint if the robot has flexibilities
   *
   * In TVM backend:
   * - mc_tvm::DynamicFunctionPtr
   */
  mc_rtc::void_ptr motion_constr_;
  /** Robot index for the constraint */
  unsigned int robotIndex_;
};

} // namespace mc_solver
