/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL, BIT
 */

#pragma once

#include <mc_solver/utils/Update.h>

namespace mc_solver
{

/** \class Constraint
 *
 * Generic helper class to write a new constraint for Tasks. \see
 * EqualityConstraint, \see InequalityConstraint and \see
 * GenInequalityConstraint for details on specific constraints you can
 * implement based on this helper
 *
 * \tparam ConstraintT Type of tasks::qp constraint
 *
 * \tparam UpdateT Type of utils::Update we will use
 */

template<typename ConstraintT, typename UpdateT>
struct MC_SOLVER_DLLAPI Constraint : public tasks::qp::ConstraintFunction<ConstraintT>, UpdateT
{
  static_assert(std::is_same<ConstraintT, tasks::qp::Equality>::value
                    || std::is_same<ConstraintT, tasks::qp::Inequality>::value
                    || std::is_same<ConstraintT, tasks::qp::GenInequality>::value,
                "This must be instanciated with a known ConstraintT");
  static_assert(utils::IsUpdate<UpdateT>::value, "This must be instanciated with a correct UpdateT parameter");

protected:
  // XXX The following is a hack to work-around the absence of constructor inheritance in GCC 4.7
  // We enable different constructor based on the UpdateT class

  template<typename U = UpdateT, typename = typename std::enable_if<std::is_same<U, utils::UpdateRobot>::value>::type>
  explicit Constraint(unsigned int rIndex) : UpdateT(rIndex)
  {
  }

  template<typename U = UpdateT, typename = typename std::enable_if<std::is_same<U, utils::UpdateLambda>::value>::type>
  explicit Constraint(const tasks::qp::ContactId & cid) : UpdateT(cid)
  {
  }

  template<typename U = UpdateT, typename = typename std::enable_if<std::is_same<U, utils::UpdateForce>::value>::type>
  Constraint(const mc_solver::QPSolver & solver, const tasks::qp::ContactId & cid) : UpdateT(solver, cid)
  {
  }

  /** Virtual destructor */
  virtual ~Constraint() {}

  void updateNrVars(const std::vector<rbd::MultiBody> & mbs, const tasks::qp::SolverData & data) override
  {
    UpdateT::updateNrVarsImpl(mbs, data);
  }

  void update(const std::vector<rbd::MultiBody> & mbs,
              const std::vector<rbd::MultiBodyConfig> & mbcs,
              const tasks::qp::SolverData & data) override
  {
    UpdateT::updateImpl(mbs, mbcs, data);
  }
};

} // namespace mc_solver
