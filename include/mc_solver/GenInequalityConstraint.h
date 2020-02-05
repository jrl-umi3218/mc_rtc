/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL, BIT
 */

#pragma once

#include <mc_solver/utils/Constraint.h>

namespace mc_solver
{

namespace utils
{

/** \class GenInequalityConstraint
 *
 * Helper class to write a general inequality constraint for Tasks.
 *
 * This implements \f$ L \le A * x \le U \f$
 *
 * You must implement the following functions:
 * - `const Eigen::MatrixXd & A() const override; // Return A`
 * - void compute() override; // Update constraint matrix and bounds
 * - int maxGenInEq() const override; // Number of inequality lines
 * - std::string nameGenInEq() const override; // Desriptive name of the constraint
 * - const Eigen::VectorXd & LowerGenInEq() const override; // Return the lower bound L
 * - const Eigen::VectorXd & UpperGenInEq() const override; // Return the upper bound U
 *
 * Depending on what the constraint applies to you have to make sure A has the
 * following size:
 * - (nrLines, alphaD.size()) for a constraint on a robot's state
 * - (nrLines, nrLambda) for a constraint on the lambdas of a t
 * - (nrLines, 6) for a constraint on a wrench
 */
template<typename UpdateT>
struct GenInequalityConstraint : public Constraint<tasks::qp::GenInequality, UpdateT>
{
  using Base = Constraint<tasks::qp::GenInequality, UpdateT>;

  // XXX The following is a hack to work-around the absence of constructor inheritance in GCC 4.7
  // We enable different constructor based on the UpdateT class

  template<typename U = UpdateT, typename = typename std::enable_if<std::is_same<U, utils::UpdateRobot>::value>::type>
  explicit GenInequalityConstraint(unsigned int rIndex) : Base(rIndex)
  {
  }

  template<typename U = UpdateT, typename = typename std::enable_if<std::is_same<U, utils::UpdateLambda>::value>::type>
  explicit GenInequalityConstraint(const tasks::qp::ContactId & cid) : Base(cid)
  {
  }

  template<typename U = UpdateT, typename = typename std::enable_if<std::is_same<U, utils::UpdateForce>::value>::type>
  GenInequalityConstraint(const mc_solver::QPSolver & solver, const tasks::qp::ContactId & cid) : Base(solver, cid)
  {
  }

  const Eigen::MatrixXd & AGenInEq() const override
  {
    return this->AFull_;
  }

  std::string descGenInEq(const std::vector<rbd::MultiBody> &, int i) override
  {
    std::stringstream ss;
    ss << "Failure at line " << i << " for inequality " << this->nameGenInEq();
    return ss.str();
  }
};

} // namespace utils

/** \class GenInequalityConstraintRobot
 *
 * Helper class to write a general inequality constraint for Tasks. This constraint
 * applies to alphaD for a given robot.
 *
 * This implements \f$L \le A * \ddot{\mathbf{q}}_{rI} \le U\f$
 *
 * Where \f$rI\f$ is the robot index you provide at construction.
 *
 * You must implement the following functions:
 * - `const Eigen::MatrixXd & A() const override;` returns \f$A\f$, must be of size (nrLines, alphaD.size())
 * - `void compute() override;` update constraint matrix and bounds
 * - `int maxGenInEq() const override;` number of inequality lines
 * - `std::string nameGenInEq() const override;` descriptive name of the constraint
 * - `const Eigen::VectorXd & LowerGenInEq() const override;` returns the lower bound \f$L\f$
 * - `const Eigen::VectorXd & UpperGenInEq() const override;` returns the upper bound \f$U\f$
 *
 */
struct GenInequalityConstraintRobot : public utils::GenInequalityConstraint<utils::UpdateRobot>
{
  explicit GenInequalityConstraintRobot(unsigned int rIndex)
  : utils::GenInequalityConstraint<utils::UpdateRobot>(rIndex)
  {
  }
};

/** \class GenInequalityConstraintLambda
 *
 * Helper class to write a general inequality constraint for
 * Tasks. This constraint applies to the vector of positive
 * multipliers (lambda) associated with a given contact
 *
 * This implements \f$L \le A * \lambda_{cI} \le U\f$
 *
 * Where \f$cI\f$ is the contact id you provide at
 * construction.
 *
 * You must implement the following functions:
 * - `const Eigen::MatrixXd & A() const override;` returns
 *   \f$A\f$, must be of size (nrLines, nrLambda)
 * - `void compute() override;` update constraint matrix and
 *   bounds
 * - `int maxGenInEq() const override;` number of inequality
 *   lines
 * - `std::string nameGenInEq() const override;` descriptive
 *   name of the constraint
 * - `const Eigen::VectorXd & LowerGenInEq() const override;`
 *   returns the lower bound \f$L\f$
 * - `const Eigen::VectorXd & UpperGenInEq() const override;`
 *   returns the upper bound \f$U\f$
 *
 */
struct GenInequalityConstraintLambda : public utils::GenInequalityConstraint<utils::UpdateLambda>
{
  explicit GenInequalityConstraintLambda(const tasks::qp::ContactId & cid)
  : utils::GenInequalityConstraint<utils::UpdateLambda>(cid)
  {
  }
};

/** \class GenInequalityConstraintForce
 *
 * Helper class to write a general inequality constraint for Tasks. This constraint
 * applies to the wrench associated with a given contact.
 *
 * This implements \f$ L \le A * \mathbf{f}_{cI} \le U \f$
 *
 * Where \f$cI\f$ is the contact id you provide at construction.
 *
 * You must implement the following functions:
 * - `const Eigen::MatrixXd & A() const override;` returns \f$A\f$, must be of size (nrLines, 6)
 * - `void compute() override;` update constraint matrix and bounds
 * - `int maxGenInEq() const override;` number of inequality lines
 * - `std::string nameGenInEq() const override;` descriptive name of the constraint
 * - `const Eigen::VectorXd & LowerGenInEq() const override;` returns the lower bound \f$L\f$
 * - `const Eigen::VectorXd & UpperGenInEq() const override;` returns the upper bound \f$U\f$
 *
 */
struct GenInequalityConstraintForce : public utils::GenInequalityConstraint<utils::UpdateForce>
{
  GenInequalityConstraintForce(const mc_solver::QPSolver & solver, const tasks::qp::ContactId & cid)
  : utils::GenInequalityConstraint<utils::UpdateForce>(solver, cid)
  {
  }
};

} // namespace mc_solver
