#pragma once

#include <mc_solver/ConstraintSet.h>
#include <mc_tvm/SSMPFLFunction.h>

#include <tvm/ControlProblem.h>

namespace mc_solver
{

/** TVM SSM + PFL speed constraint with a fixed number of pair rows. */
struct MC_SOLVER_DLLAPI SSMPFLConstraint : public ConstraintSet
{
  using RegionParameters = mc_tvm::SSMPFLFunction::RegionParameters;
  using Measurement = mc_tvm::SSMPFLFunction::Measurement;
  using RowData = mc_tvm::SSMPFLFunction::RowData;

  SSMPFLConstraint(const mc_rbdyn::Robot & robot,
                   unsigned int capacity,
                   double dt,
                   std::map<std::string, RegionParameters> regions,
                   double stoppingTime,
                   double minMobility = 1e-12,
                   double minMassEigenvalue = 1e-9,
                   double weight = 0);

  void setMeasurements(const std::vector<Measurement> & measurements);
  void clear();
  double softSlack(size_t row) const;

  unsigned int capacity() const noexcept { return function_->capacity(); }
  const std::map<std::string, RegionParameters> & regions() const noexcept { return function_->regions(); }
  const std::vector<RowData> & rowData() const noexcept { return function_->rowData(); }
  const Eigen::MatrixXd & rows() const noexcept { return function_->rows(); }
  const Eigen::VectorXd & bounds() const noexcept { return function_->bounds(); }

  void addToSolverImpl(QPSolver & solver) override;
  void removeFromSolverImpl(QPSolver & solver) override;

private:
  double weight_;
  tvm::VariablePtr slack_;
  mc_tvm::SSMPFLFunctionPtr function_;
  tvm::TaskWithRequirementsPtr task_;
  tvm::TaskWithRequirementsPtr slackBound_;
  tvm::TaskWithRequirementsPtr slackObjective_;
};

} // namespace mc_solver
