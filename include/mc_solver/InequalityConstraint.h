/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL, BIT
 */

#pragma once

#include <mc_solver/utils/Constraint.h>

namespace mc_solver
{

namespace utils
{

/** \class InequalityConstraint
 *
 * Helper class to write an inequality constraint for Tasks.
 *
 * This implements \f$ A * x \le b \f$
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
struct InequalityConstraint : public Constraint<tasks::qp::Inequality, UpdateT>
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

} // namespace utils

/** \class InequalityConstraintRobot
 *
 * Helper class to write an inequality constraint for Tasks. This constraint
 * applies to alphaD for a given robot.
 *
 * This implements \f$ A * \ddot{\mathbf{q}}_{rI} \le b \f$
 *
 * Where \f$rI\f$ is the robot index you provide at construction.
 *
 * You must implement the following functions:
 * - `const Eigen::MatrixXd & A() const override;` returns \f$A\f$, must be of size (nrLines, alphaD.size())
 * - `void compute() override;` update constraint matrix and bounds
 * - `int maxInEq() const override;` number of inequality lines
 * - `std::string nameInEq() const override;` descriptive name of the constraint
 * - `const Eigen::VectorXd & bInEq() const override;` returns \f$b\f$
 *
 */
struct InequalityConstraintRobot : public utils::InequalityConstraint<utils::UpdateRobot>
{
  explicit InequalityConstraintRobot(unsigned int rIndex) : utils::InequalityConstraint<utils::UpdateRobot>(rIndex) {}
};

/** \class InequalityConstraintLambda
 *
 * Helper class to write an inequality constraint for Tasks.
 * This constraint applies to the vector of positive
 * multipliers (lambda) associated with a given contact
 *
 * This implements \f$ A * \lambda_{cI} \le b \f$
 *
 * Where \f$cI\f$ is the contact id you provide at
 * construction.
 *
 * You must implement the following functions:
 * - `const Eigen::MatrixXd & A() const override;` returns
 *   \f$A\f$, must be of size (nrLines, nrLambda)
 * - `void compute() override;` update constraint matrix and
 *   bounds
 * - `int maxInEq() const override;` number of inequality
 *   lines
 * - `std::string nameInEq() const override;` descriptive name
 *   of the constraint
 * - `const Eigen::VectorXd & bInEq() const override;` returns
 *   \f$b\f$
 *
 */
struct InequalityConstraintLambda : public utils::InequalityConstraint<utils::UpdateLambda>
{
  explicit InequalityConstraintLambda(const tasks::qp::ContactId & cid)
  : utils::InequalityConstraint<utils::UpdateLambda>(cid)
  {
  }
};

/** \class InequalityConstraintForce
 *
 * Helper class to write an inequality constraint for Tasks. This constraint
 * applies to the wrench associated with a given contact.
 *
 * The wrench \f$ \mathbf{f}_{cI} \f$ is expressed in the first robot (`cid.r1Index`)
 * contact surface frame.
 *
 * This implements \f$ A * \mathbf{f}_{cI} \le b \f$
 *
 * Where \f$cI\f$ is the contact id you provide at construction.
 *
 * You must implement the following functions:
 * - `const Eigen::MatrixXd & A() const override;` returns \f$A\f$, must be of size (nrLines, 6)
 * - `void compute() override;` update constraint matrix and bounds
 * - `int maxInEq() const override;` number of inequality lines
 * - `std::string nameInEq() const override;` descriptive name of the constraint
 * - `const Eigen::VectorXd & bInEq() const override;` returns \f$b\f$
 *
 */
struct InequalityConstraintForce : public utils::InequalityConstraint<utils::UpdateForce>
{
  InequalityConstraintForce(const mc_solver::QPSolver & solver, const tasks::qp::ContactId & cid)
  : utils::InequalityConstraint<utils::UpdateForce>(solver, cid)
  {
  }
};

} // namespace mc_solver
