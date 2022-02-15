/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL, BIT
 */

#include <mc_rtc/logging.h>
#include <mc_solver/utils/UpdateNrVars.h>

namespace mc_solver
{

namespace utils
{

UpdateNrVarsRobot::UpdateNrVarsRobot(unsigned int rIndex) : rIndex_(rIndex) {}

void UpdateNrVarsRobot::updateNrVarsImpl(const std::vector<rbd::MultiBody> &, const tasks::qp::SolverData & data)
{
  nrVars_ = data.nrVars();
  ABegin_ = data.alphaDBegin(static_cast<int>(rIndex_));
}

UpdateNrVarsLambda::UpdateNrVarsLambda(const tasks::qp::ContactId & cid) : cid_(cid) {}

void UpdateNrVarsLambda::updateNrVarsImpl(const std::vector<rbd::MultiBody> &, const tasks::qp::SolverData & data)
{
  nrVars_ = data.nrVars();
  ABegin_ = -1;
  for(size_t i = 0; i < data.allContacts().size(); ++i)
  {
    const auto & c = data.allContacts()[i];
    if(c.contactId == cid_)
    {
      ABegin_ = data.lambdaBegin(static_cast<int>(i));
      break;
    }
  }
  if(ABegin_ == -1)
  {
    mc_rtc::log::error_and_throw("Generic InequalityConstraint added for a contact that is not in the solver");
  }
}

} // namespace utils

} // namespace mc_solver
