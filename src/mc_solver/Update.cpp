/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL, BIT
 */

#include <mc_solver/utils/Update.h>

namespace mc_solver
{

namespace utils
{

UpdateForce::UpdateForce(const mc_solver::QPSolver & solver, const tasks::qp::ContactId & cid)
: Update<UpdateNrVarsLambda>(cid), transform_(solver, cid)
{
}

void UpdateForce::updateImpl(const std::vector<rbd::MultiBody> &,
                             const std::vector<rbd::MultiBodyConfig> &,
                             const tasks::qp::SolverData &)
{
  compute();
  const auto & A_ = A();
  ALambda_.noalias() = A_ * transform_.transform();
  auto nInEq = A_.rows();
  AFull_.setZero(nInEq, nrVars_);
  AFull_.block(0, ABegin_, nInEq, A_.cols()) = ALambda_;
}

} // namespace utils

} // namespace mc_solver
