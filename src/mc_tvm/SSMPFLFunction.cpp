#include <mc_tvm/SSMPFLFunction.h>

#include <mc_rbdyn/Robot.h>
#include <mc_tvm/Robot.h>

#include <RBDyn/Jacobian.h>
#include <RBDyn/MultiBodyConfig.h>

#include <Eigen/Cholesky>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mc_tvm
{

SSMPFLFunction::SSMPFLFunction(const mc_rbdyn::Robot & robot,
                               unsigned int capacity,
                               double dt,
                               std::map<std::string, RegionParameters> regions,
                               double stoppingTime,
                               double minMobility,
                               double minMassEigenvalue,
                               tvm::VariablePtr slack)
: tvm::function::abstract::LinearFunction(static_cast<int>(capacity)), robot_(robot), capacity_(capacity), dt_(dt),
  regions_(std::move(regions)), stoppingTime_(stoppingTime), minMobility_(minMobility),
  minMassEigenvalue_(minMassEigenvalue), slack_(std::move(slack)), inertia_(robot.mb()),
  rawA_(Eigen::MatrixXd::Zero(capacity, robot.mb().nrDof())), rawUpper_(Eigen::VectorXd::Zero(capacity)),
  scaledA_(rawA_), slackA_(Eigen::MatrixXd::Zero(capacity, capacity)), scaledUpper_(rawUpper_)
{
  if(capacity == 0 || !std::isfinite(dt) || dt <= 0 || !std::isfinite(stoppingTime) || stoppingTime <= 0
     || !std::isfinite(minMobility) || minMobility < 0 || !std::isfinite(minMassEigenvalue) || minMassEigenvalue <= 0)
    throw std::invalid_argument("invalid SSM+PFL function settings");
  for(const auto & [name, p] : regions_)
  {
    if(name.empty() || !std::isfinite(p.forceMaxN) || p.forceMaxN < 0 || !std::isfinite(p.stiffnessNPerM)
       || p.stiffnessNPerM <= 0 || !std::isfinite(p.distanceMarginM) || p.distanceMarginM < 0)
      throw std::invalid_argument("invalid SSM+PFL region parameters");
  }
  if(regions_.empty()) { throw std::invalid_argument("SSM+PFL requires at least one region"); }
  if(slack_ && slack_->size() != static_cast<int>(capacity))
    throw std::invalid_argument("SSM+PFL slack size does not match capacity");

  registerUpdates(Update::Jacobian, &SSMPFLFunction::updateJacobian, Update::B, &SSMPFLFunction::updateb);
  addOutputDependency<SSMPFLFunction>(Output::Jacobian, Update::Jacobian);
  addOutputDependency<SSMPFLFunction>(Output::B, Update::B);
  addVariable(robot.tvmRobot().alphaD(), true);
  if(slack_) { addVariable(slack_, true); }
  velocity_.setZero();
}

void SSMPFLFunction::clear()
{
  rawA_.setZero();
  rawUpper_.setZero();
  scaledA_.setZero();
  slackA_.setZero();
  scaledUpper_.setZero();
  rowData_.clear();
}

void SSMPFLFunction::setMeasurements(const std::vector<Measurement> & measurements)
{
  if(measurements.size() > capacity_) { throw std::invalid_argument("too many SSM+PFL measurements"); }
  if(measurements.empty()) { clear(); return; }
  const auto & mb = robot_.mb();
  const auto & mbc = robot_.mbc();
  const int dofs = mb.nrDof();
  const Eigen::VectorXd qdot = rbd::dofToVector(mb, robot_.alpha());
  if(qdot.size() != dofs || !qdot.allFinite()) { throw std::domain_error("invalid robot velocity"); }
  inertia_.computeH(mb, mbc);
  const Eigen::MatrixXd & mass = inertia_.H();
  if(mass.rows() != dofs || mass.cols() != dofs || !mass.allFinite()) throw std::domain_error("invalid robot inertia");
  const Eigen::LDLT<Eigen::MatrixXd> factor(mass);
  if(factor.info() != Eigen::Success || factor.vectorD().minCoeff() < minMassEigenvalue_)
    throw std::domain_error("robot inertia is not positive definite");

  Eigen::MatrixXd nextRawA = Eigen::MatrixXd::Zero(capacity_, dofs);
  Eigen::VectorXd nextRawUpper = Eigen::VectorXd::Zero(capacity_);
  Eigen::MatrixXd nextScaledA = nextRawA;
  Eigen::MatrixXd nextSlackA = Eigen::MatrixXd::Zero(capacity_, capacity_);
  Eigen::VectorXd nextScaledUpper = nextRawUpper;
  std::vector<RowData> nextRows;
  nextRows.reserve(measurements.size());
  for(size_t i = 0; i < measurements.size(); ++i)
  {
    const auto & m = measurements[i];
    const auto region = regions_.find(m.region);
    if(region == regions_.end() || !std::isfinite(m.distance) || m.distance < 0 || !m.robotPointBodyLocal.allFinite()
       || !m.normal.allFinite() || !m.humanVelocity.allFinite())
      throw std::domain_error("invalid SSM+PFL pair measurement");
    const double normalNorm = m.normal.norm();
    if(!std::isfinite(normalNorm) || normalNorm < 1e-9) throw std::domain_error("invalid SSM+PFL pair normal");
    const Eigen::Vector3d normal = m.normal / normalNorm;
    rbd::Jacobian jacobian(mb, m.robotBody, m.robotPointBodyLocal);
    Eigen::MatrixXd full(6, dofs), fullDot(6, dofs);
    jacobian.fullJacobian(mb, jacobian.jacobian(mb, mbc), full);
    jacobian.fullJacobian(mb, jacobian.jacobianDot(mb, mbc), fullDot);
    if(!full.allFinite() || !fullDot.allFinite()) throw std::domain_error("invalid SSM+PFL point Jacobian");
    RowData row;
    row.directionalJacobian = normal.transpose() * full.bottomRows(3);
    const double mobility = row.directionalJacobian.dot(factor.solve(row.directionalJacobian.transpose()));
    if(!std::isfinite(mobility) || mobility < -minMobility_)
      throw std::domain_error("invalid SSM+PFL directional inverse mass");
    const auto & p = region->second;
    const double pflSpeed = p.forceMaxN / std::sqrt(p.stiffnessNPerM) * std::sqrt(std::max(0.0, mobility));
    const double combinedSpeed = std::max(m.distance - p.distanceMarginM, 0.0) / stoppingTime_ + pflSpeed;
    row.robotSpeedBudget = std::max(0.0, combinedSpeed + normal.dot(m.humanVelocity));
    row.robotApproach = row.directionalJacobian.dot(qdot);
    row.jacobianDrift = (normal.transpose() * fullDot.bottomRows(3) * qdot).value();
    row.accelerationBound = row.robotSpeedBudget - row.robotApproach - dt_ * row.jacobianDrift;
    if(!std::isfinite(pflSpeed) || !std::isfinite(combinedSpeed) || !std::isfinite(row.robotSpeedBudget)
       || !std::isfinite(row.robotApproach) || !std::isfinite(row.jacobianDrift)
       || !std::isfinite(row.accelerationBound))
      throw std::domain_error("nonfinite SSM+PFL row");
    nextRawA.row(static_cast<int>(i)) = dt_ * row.directionalJacobian;
    nextRawUpper[static_cast<int>(i)] = row.accelerationBound;
    const double scale = 1.0 / std::max(1e-3, nextRawA.row(static_cast<int>(i)).norm());
    nextScaledA.row(static_cast<int>(i)) = scale * nextRawA.row(static_cast<int>(i));
    nextScaledUpper[static_cast<int>(i)] = scale * row.accelerationBound;
    if(slack_) { nextSlackA(static_cast<int>(i), static_cast<int>(i)) = -scale; }
    nextRows.push_back(std::move(row));
  }
  rawA_.swap(nextRawA);
  rawUpper_.swap(nextRawUpper);
  scaledA_.swap(nextScaledA);
  slackA_.swap(nextSlackA);
  scaledUpper_.swap(nextScaledUpper);
  rowData_.swap(nextRows);
}

void SSMPFLFunction::updateJacobian()
{
  jacobian_[robot_.tvmRobot().alphaD().get()] = scaledA_;
  if(slack_) { jacobian_[slack_.get()] = slackA_; }
}

void SSMPFLFunction::updateb()
{
  b_ = -scaledUpper_;
}

} // namespace mc_tvm
