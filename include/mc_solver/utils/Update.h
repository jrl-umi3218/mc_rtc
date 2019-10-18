/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL, BIT
 */

#pragma once

#include <mc_solver/utils/ContactWrenchMatrixToLambdaMatrix.h>
#include <mc_solver/utils/UpdateNrVars.h>

namespace mc_solver
{

namespace utils
{

struct MC_SOLVER_DLLAPI UpdateTag
{
};

template<typename UpdateNrVars>
struct MC_SOLVER_DLLAPI Update : public UpdateNrVars, UpdateTag
{
  static_assert(std::is_same<UpdateNrVars, UpdateNrVarsRobot>::value
                    || std::is_same<UpdateNrVars, UpdateNrVarsLambda>::value,
                "This should not be instanciated with unexpected UpdateNrVars classes");

protected:
  // XXX The following is a hack to work-around the absence of constructor inheritance in GCC 4.7
  // We enable different constructor based on the UpdateNrVars class

  template<typename U = UpdateNrVars,
           typename = typename std::enable_if<std::is_same<U, UpdateNrVarsRobot>::value>::type>
  explicit Update(unsigned int rIndex) : UpdateNrVars(rIndex)
  {
  }

  template<typename U = UpdateNrVars,
           typename = typename std::enable_if<std::is_same<U, UpdateNrVarsLambda>::value>::type>
  explicit Update(const tasks::qp::ContactId & cid) : UpdateNrVars(cid)
  {
  }

  virtual ~Update() {}

  /** Should be overriden to update the constraint matrix and vector(s) */
  virtual void compute() = 0;

  /** Should be overriden to provide A in the desired operation space */
  virtual const Eigen::MatrixXd & A() const = 0;

  void updateImpl(const std::vector<rbd::MultiBody> &,
                  const std::vector<rbd::MultiBodyConfig> &,
                  const tasks::qp::SolverData &)
  {
    compute();
    const auto & A_ = A();
    auto nInEq = A_.rows();
    AFull_.setZero(nInEq, UpdateNrVars::nrVars_);
    AFull_.block(0, UpdateNrVars::ABegin_, nInEq, A_.cols()) = A_;
  }

  /** Holds the full size matrix */
  Eigen::MatrixXd AFull_;
};

template<typename T>
struct IsUpdate : public std::is_base_of<UpdateTag, T>
{
};

using UpdateRobot = Update<UpdateNrVarsRobot>;
using UpdateLambda = Update<UpdateNrVarsLambda>;

struct MC_SOLVER_DLLAPI UpdateForce : public Update<UpdateNrVarsLambda>
{
protected:
  UpdateForce(const mc_solver::QPSolver & solver, const tasks::qp::ContactId & cid);

  void updateImpl(const std::vector<rbd::MultiBody> &,
                  const std::vector<rbd::MultiBodyConfig> &,
                  const tasks::qp::SolverData &);

private:
  /** Holds transformation from force matrix to lambda matrix */
  ContactWrenchMatrixToLambdaMatrix transform_;

  Eigen::MatrixXd ALambda_;
};

} // namespace utils

} // namespace mc_solver
