/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL, BIT
 */

#pragma once

#include <mc_solver/Constraint.h>

namespace mc_solver
{

/** \class InequalityConstraint
 *
 * Helper class to write an inequality constraint for Tasks.
 *
 * This implements \f$ A * x <= b \f$
 *
 * You must implement the following functions:
 * - const Eigen::MatrixXd & A() const override; // Return A
 * - void compute() override; // Update constraint matrix and bounds
 * - int maxInEq() const override; // Number of equality lines
 * - std::string nameInEq() const override; // Desriptive name of the constraint
 * - const Eigen::VectorXd & bInEq() const override; // Return the bound b
 *
 * Depending on what the constraint apply to you have to make sure A has the
 * following size:
 * - (nrLines, alphaD.size()) for a constraint on a robot's state
 * - (nrLines, nrLambda) for a constraint on the lambdas of a contact
 * - (nrLines, 6) for a constraint on a wrench
 */
template<typename UpdateT>
struct MC_SOLVER_DLLAPI InequalityConstraint : public Constraint<tasks::qp::Inequality, UpdateT>
{
  using Base = Constraint<tasks::qp::Inequality, UpdateT>;

  // XXX The following is a hack to work-around the absence of constructor inheritance in GCC 4.7
  // We enable different constructor based on the UpdateT class

  template<typename U = UpdateT, typename = typename std::enable_if<std::is_same<U, utils::UpdateRobot>::value>::type>
  explicit InequalityConstraint(unsigned int rIndex) : Base(rIndex)
  {
  }

  template<typename U = UpdateT, typename = typename std::enable_if<std::is_same<U, utils::UpdateLambda>::value>::type>
  explicit InequalityConstraint(const tasks::qp::ContactId & cid) : Base(cid)
  {
  }

  template<typename U = UpdateT, typename = typename std::enable_if<std::is_same<U, utils::UpdateForce>::value>::type>
  InequalityConstraint(const mc_solver::QPSolver & solver, const tasks::qp::ContactId & cid) : Base(solver, cid)
  {
  }

  const Eigen::MatrixXd & AInEq() const override
  {
    return this->AFull_;
  }

  std::string descInEq(const std::vector<rbd::MultiBody> &, int i) override
  {
    std::stringstream ss;
    ss << "Failure at line " << i << " for inequality " << this->nameInEq();
    return ss.str();
  }
};

using InequalityConstraintRobot = InequalityConstraint<utils::UpdateRobot>;
using InequalityConstraintLambda = InequalityConstraint<utils::UpdateLambda>;
using InequalityConstraintForce = InequalityConstraint<utils::UpdateForce>;

} // namespace mc_solver
