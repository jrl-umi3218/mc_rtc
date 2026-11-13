#pragma once

#include <mc_tvm/api.h>

#include <tvm/Variable.h>
#include <tvm/function/abstract/LinearFunction.h>

#include <RBDyn/FD.h>
#include <Eigen/Core>

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace mc_rbdyn
{
struct Robot;
}

namespace mc_tvm
{

/** Fixed-size, pointwise SSM + PFL acceleration inequality for one robot.
 * A row is dt * J_n * alphaD <= v_budget - J_n * alpha - dt * Jdot_n * alpha.
 */
class MC_TVM_DLLAPI SSMPFLFunction : public tvm::function::abstract::LinearFunction
{
public:
  using Output = tvm::function::abstract::LinearFunction::Output;
  DISABLE_OUTPUTS(Output::JDot)
  SET_UPDATES(SSMPFLFunction, Jacobian, B)

  struct RegionParameters
  {
    double forceMaxN = 0;
    double stiffnessNPerM = 0;
    double distanceMarginM = 0;
  };

  struct Measurement
  {
    std::string region;
    std::string robotBody;
    Eigen::Vector3d robotPointBodyLocal = Eigen::Vector3d::Zero();
    /** Unit direction from the robot witness toward the human witness. */
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    Eigen::Vector3d humanVelocity = Eigen::Vector3d::Zero();
    double distance = 0;
  };

  struct RowData
  {
    Eigen::RowVectorXd directionalJacobian;
    double robotApproach = 0;
    double jacobianDrift = 0;
    double robotSpeedBudget = 0;
    double accelerationBound = 0;
  };

  SSMPFLFunction(const mc_rbdyn::Robot & robot,
                 unsigned int capacity,
                 double dt,
                 std::map<std::string, RegionParameters> regions,
                 double stoppingTime,
                 double minMobility,
                 double minMassEigenvalue,
                 tvm::VariablePtr slack = nullptr);

  /** Recompute all rows atomically before the next QP solve. */
  void setMeasurements(const std::vector<Measurement> & measurements);
  void clear();

  unsigned int capacity() const noexcept { return capacity_; }
  const std::map<std::string, RegionParameters> & regions() const noexcept { return regions_; }
  const std::vector<RowData> & rowData() const noexcept { return rowData_; }
  const Eigen::MatrixXd & rows() const noexcept { return rawA_; }
  const Eigen::VectorXd & bounds() const noexcept { return rawUpper_; }

protected:
  void updateJacobian();
  void updateb();

private:
  const mc_rbdyn::Robot & robot_;
  const unsigned int capacity_;
  const double dt_;
  const std::map<std::string, RegionParameters> regions_;
  const double stoppingTime_;
  const double minMobility_;
  const double minMassEigenvalue_;
  tvm::VariablePtr slack_;
  rbd::ForwardDynamics inertia_;
  Eigen::MatrixXd rawA_;
  Eigen::VectorXd rawUpper_;
  Eigen::MatrixXd scaledA_;
  Eigen::MatrixXd slackA_;
  Eigen::VectorXd scaledUpper_;
  std::vector<RowData> rowData_;
};

using SSMPFLFunctionPtr = std::shared_ptr<SSMPFLFunction>;

} // namespace mc_tvm
