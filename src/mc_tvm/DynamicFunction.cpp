/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/DynamicFunction.h>

#include <mc_tvm/Robot.h>
#include <mc_tvm/RobotFrame.h>

namespace mc_tvm
{

DynamicFunction::DynamicFunction(const mc_rbdyn::Robot & robot, bool compensateExternalForces)
: tvm::function::abstract::LinearFunction(robot.mb().nrDof()), robot_(robot),
  compensateExternalForces_(compensateExternalForces)
{
  registerUpdates(Update::B, &DynamicFunction::updateb);
  registerUpdates(Update::Jacobian, &DynamicFunction::updateJacobian);
  addOutputDependency<DynamicFunction>(Output::B, Update::B);
  addOutputDependency<DynamicFunction>(Output::Jacobian, Update::Jacobian);
  auto & tvm_robot = robot.tvmRobot();
  addInputDependency<DynamicFunction>(Update::Jacobian, tvm_robot, Robot::Output::H);
  addInputDependency<DynamicFunction>(Update::B, tvm_robot, Robot::Output::C);
  if(compensateExternalForces_)
  {
    addInputDependency<DynamicFunction>(Update::B, tvm_robot, Robot::Output::ExternalForces);
  }
  addVariable(tvm::dot(tvm_robot.q(), 2), true);
  addVariable(tvm_robot.tau(), true);
  jacobian_[tvm_robot.tau().get()] = -Eigen::MatrixXd::Identity(robot_.mb().nrDof(), robot_.mb().nrDof());
  jacobian_[tvm_robot.tau().get()].properties(tvm::internal::MatrixProperties::MINUS_IDENTITY);
  velocity_.setZero();
}

DynamicFunction::ForceContact::ForceContact(const mc_rbdyn::RobotFrame & frame,
                                            std::vector<sva::PTransformd> points,
                                            double dir)
: frame_(frame), points_(std::move(points)), dir_(dir), jac_(frame.tvm_frame().rbdJacobian()),
  blocks_(jac_.compactPath(frame.robot().mb())), force_jac_(6, jac_.dof()), full_jac_(6, frame.robot().mb().nrDof())
{
  for(size_t i = 0; i < points_.size(); ++i) { forces_.add(tvm::Space(3).createVariable("force" + std::to_string(i))); }
  forces_.setZero();
}

void DynamicFunction::ForceContact::updateJacobians(DynamicFunction & parent)
{
  const auto & robot = frame_->robot();
  const auto & bodyJac = jac_.bodyJacobian(robot.mb(), robot.mbc());
  for(int i = 0; i < forces_.numberOfVariables(); ++i)
  {
    const auto & force = forces_[i];
    const auto & point = points_[static_cast<size_t>(i)];
    jac_.translateBodyJacobian(bodyJac, robot.mbc(), point.translation(), force_jac_);
    full_jac_.setZero();
    jac_.addFullJacobian(blocks_, force_jac_, full_jac_);
    parent.jacobian_[force.get()].noalias() = -dir_ * full_jac_.block(3, 0, 3, robot.mb().nrDof()).transpose();
  }
}

sva::ForceVecd DynamicFunction::ForceContact::force() const
{
  sva::ForceVecd ret = sva::ForceVecd::Zero();
  for(int i = 0; i < forces_.numberOfVariables(); ++i)
  {
    const auto & force = forces_[i];
    const auto & point = points_[static_cast<size_t>(i)];
    ret += point.transMul(sva::ForceVecd(Eigen::Vector3d::Zero(), force->value()));
  }
  return ret;
}

const tvm::VariableVector & DynamicFunction::addContact(const mc_rbdyn::RobotFrame & frame,
                                                        std::vector<sva::PTransformd> points,
                                                        double dir)
{
  if(frame.robot().name() != robot_.name())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "Attempted to add a contact for {} to dynamic function belonging to {}", frame.robot().name(), robot_.name());
  }
  auto & fc = contacts_.emplace_back(frame, std::move(points), dir);
  for(const auto & var : fc.forces_) { addVariable(var, true); }
  addInputDependency<DynamicFunction>(Update::Jacobian, frame.tvm_frame(), mc_tvm::RobotFrame::Output::Jacobian);
  return fc.forces_;
}

void DynamicFunction::removeContact(const mc_rbdyn::RobotFrame & frame)
{
  auto it = findContact(frame);
  if(it != contacts_.end())
  {
    for(const auto & var : it->forces_) { removeVariable(var); }
    contacts_.erase(it);
  }
}

sva::ForceVecd DynamicFunction::contactForce(const mc_rbdyn::RobotFrame & frame) const
{
  auto it = findContact(frame);
  if(it != contacts_.end()) { return (*it).force(); }
  else
  {
    mc_rtc::log::error("No contact at frame {} in dynamic function for {}", frame.name(), robot_.name());
    return sva::ForceVecd(Eigen::Vector6d::Zero());
  }
}

void DynamicFunction::updateb()
{
  b_ = robot_.tvmRobot().C();
  if(compensateExternalForces_)
  {
    b_ -= robot_.tvmRobot().tauExternal();
    // mc_rtc::log::info("Compensating for : {}", robot_.tvmRobot().tauExternal().transpose());
    if(robot_.hasDevice<mc_rbdyn::VirtualTorqueSensor>("virtualTorqueSensor"))
    {
      b_ += robot_.device<mc_rbdyn::VirtualTorqueSensor>("virtualTorqueSensor").torques();
    }
  }
}

void DynamicFunction::updateJacobian()
{
  const auto & robot = robot_.tvmRobot();
  splitJacobian(robot.H(), robot.alphaD());
  for(auto & c : contacts_) { c.updateJacobians(*this); }
}

auto DynamicFunction::findContact(const mc_rbdyn::RobotFrame & frame) const -> std::vector<ForceContact>::const_iterator
{
  return std::find_if(contacts_.begin(), contacts_.end(), [&](const auto & c) { return c.frame_.get() == &frame; });
}

} // namespace mc_tvm
