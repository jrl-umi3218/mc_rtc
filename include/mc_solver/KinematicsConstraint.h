/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Robots.h>
#include <mc_solver/ConstraintSet.h>

#include <mc_rtc/void_ptr.h>

namespace mc_solver
{

/** \class KinematicsConstraint
 * Holds kinematic constraints (joint limits) for the robot
 */

struct MC_SOLVER_DLLAPI KinematicsConstraint : public ConstraintSet
{
public:
  /** Regular constraint constructor
   * Builds a regular joint limits constraint, prefer a damped joint limits
   * constraint in general
   * See tasks::qp::JointLimitsConstr for detail
   * \param robots The robots including the robot affected by this constraint
   * \param robotIndex The index of the robot affected by this constraint
   * \param timeStep Solver timestep
   */
  KinematicsConstraint(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double timeStep);

  /** Damped constraint constructor
   * Builds a damped joint limits constraint
   * See tasks::qp::DamperJointLimitsConstr documentation for detail
   * \param robots The robots including the robot affected by this constraint
   * \param robotIndex The index of the robot affected by this constraint
   * \param timeStep Solver timestep
   * \param damper Value of the damper {interaction distance, safety distance,
   * offset}
   * \param velocityPercent Maximum joint velocity percentage, 0.5 is advised
   */
  KinematicsConstraint(const mc_rbdyn::Robots & robots,
                       unsigned int robotIndex,
                       double timeStep,
                       const std::array<double, 3> & damper,
                       double velocityPercent = 0.5);

protected:
  /** Implementation of mc_solver::ConstraintSet::addToSolver */
  void addToSolverImpl(mc_solver::QPSolver & solver) override;
  /** Implementation of mc_solver::ConstraintSet::removeFromSolver */
  void removeFromSolverImpl(mc_solver::QPSolver & solver) override;
  /** Holds the constraint implementation
   *
   * In Tasks backend:
   * - tasks::qp::JointLimitsConstr for non-damped constraint
   * - tasks::qp::DampedJointLimitsConstr for damped-constraint
   *
   * In TVM backend:
   * - internal implementation
   *
   * The deleter carries the initial type of the constraint
   */
  mc_rtc::void_ptr constraint_;
};

} // namespace mc_solver
