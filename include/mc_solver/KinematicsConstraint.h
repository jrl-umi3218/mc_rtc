#pragma once

#include <mc_solver/ConstraintSet.h>

#include <mc_rbdyn/Robots.h>

#include <Tasks/QPConstr.h>

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
  KinematicsConstraint(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double timeStep, const std::array<double, 3> & damper, double velocityPercent = 1.0);

  /** Implementation of mc_solver::ConstraintSet::addToSolver */
  virtual void addToSolver(const std::vector<rbd::MultiBody> & mbs, tasks::qp::QPSolver & solver) override;

  /** Implementation of mc_solver::ConstraintSet::removeFromSolver */
  virtual void removeFromSolver(tasks::qp::QPSolver & solver) override;
public:
  /** Holds JointLimitsConstr, can be null depending on construction */
  std::shared_ptr<tasks::qp::JointLimitsConstr> jointLimitsConstr;
  /** Holds DamperJointLimitsConstr, can be null depending on construction */
  std::shared_ptr<tasks::qp::DamperJointLimitsConstr> damperJointLimitsConstr;
public:
  /** \deprecated{Default constructor, not made for general usage} */
  KinematicsConstraint() {}
};


} // namespace mc_solver
