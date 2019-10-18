/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL, BIT
 */

#pragma once

#include <mc_solver/Constraint.h>

namespace mc_solver
{

/** \class EqualityConstraint
 *
 * Helper class to write an equality constraint for Tasks.
 *
 * This implements \f$ A * x = b \f$
 *
 * You must implement the following functions:
 * - const Eigen::MatrixXd & A() const override; // Return A
 * - void compute() override; // Update constraint matrix and bounds
 * - int maxEq() const override; // Number of equality lines
 * - std::string nameEq() const override; // Desriptive name of the constraint
 * - const Eigen::VectorXd & bEq() const override; // Return the right-side b
 *
 * Depending on what the constraint apply to you have to make sure A has the
 * following size:
 * - (nrLines, alphaD.size()) for a constraint on a robot's state
 * - (nrLines, nrLambda) for a constraint on the lambdas of a contact
 * - (nrLines, 6) for a constraint on a wrench
 */
template<typename UpdateT>
struct MC_SOLVER_DLLAPI EqualityConstraint : public Constraint<tasks::qp::Equality, UpdateT>
{
  using Base = Constraint<tasks::qp::Equality, UpdateT>;

  // XXX The following is a hack to work-around the absence of constructor inheritance in GCC 4.7
  // We enable different constructor based on the UpdateT class

  template<typename U = UpdateT, typename = typename std::enable_if<std::is_same<U, utils::UpdateRobot>::value>::type>
  explicit EqualityConstraint(unsigned int rIndex) : Base(rIndex)
  {
  }

  template<typename U = UpdateT, typename = typename std::enable_if<std::is_same<U, utils::UpdateLambda>::value>::type>
  explicit EqualityConstraint(const tasks::qp::ContactId & cid) : Base(cid)
  {
  }

  template<typename U = UpdateT, typename = typename std::enable_if<std::is_same<U, utils::UpdateForce>::value>::type>
  EqualityConstraint(const mc_solver::QPSolver & solver, const tasks::qp::ContactId & cid) : Base(solver, cid)
  {
  }

  const Eigen::MatrixXd & AEq() const override
  {
    return this->AFull_;
  }

  std::string descEq(const std::vector<rbd::MultiBody> &, int i) override
  {
    std::stringstream ss;
    ss << "Failure at line " << i << " for inequality " << this->nameEq();
    return ss.str();
  }
};

using EqualityConstraintRobot = EqualityConstraint<utils::UpdateRobot>;
using EqualityConstraintLambda = EqualityConstraint<utils::UpdateLambda>;
using EqualityConstraintForce = EqualityConstraint<utils::UpdateForce>;

} // namespace mc_solver
