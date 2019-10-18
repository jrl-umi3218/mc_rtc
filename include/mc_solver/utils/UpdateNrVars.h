/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL, BIT
 */

#pragma once

#include <mc_solver/api.h>

#include <Tasks/QPSolver.h>

namespace mc_solver
{

namespace utils
{

/** This struct provides common data for UpdateNrVars* classes */
struct MC_SOLVER_DLLAPI UpdateNrVarsData
{
protected:
  int nrVars_;
  int ABegin_;
};

/** This struct provides an implementation for updateNrVars for a robot-based constraint */
struct MC_SOLVER_DLLAPI UpdateNrVarsRobot : public UpdateNrVarsData
{
protected:
  explicit UpdateNrVarsRobot(unsigned int rIndex);

  void updateNrVarsImpl(const std::vector<rbd::MultiBody> & mbs, const tasks::qp::SolverData & data);

  unsigned int rIndex_;
};

/** This struct provides an implement for updateNrVars for a lambda-based constraint */
struct MC_SOLVER_DLLAPI UpdateNrVarsLambda : public UpdateNrVarsData
{
protected:
  explicit UpdateNrVarsLambda(const tasks::qp::ContactId & cid);

  void updateNrVarsImpl(const std::vector<rbd::MultiBody> & mbs, const tasks::qp::SolverData & data);

  tasks::qp::ContactId cid_;
};

} // namespace utils

} // namespace mc_solver
