/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/DynamicFunction.h>

#include <mc_rbdyn/Contact.h>
#include <mc_tvm/Robot.h>
#include <mc_tvm/RobotFrame.h>

namespace mc_tvm
{

DynamicFunction::DynamicFunction(const mc_rbdyn::Robot & robot)
: tvm::function::abstract::LinearFunction(robot.mb().nrDof()), robot_(robot)
{
  registerUpdates(Update::B, &DynamicFunction::updateb);
  registerUpdates(Update::Jacobian, &DynamicFunction::updateJacobian);
  addOutputDependency<DynamicFunction>(Output::B, Update::B);
  addOutputDependency<DynamicFunction>(Output::Jacobian, Update::Jacobian);
  auto & tvm_robot = robot.tvmRobot();
  addInputDependency<DynamicFunction>(Update::Jacobian, tvm_robot, Robot::Output::H);
  addInputDependency<DynamicFunction>(Update::B, tvm_robot, Robot::Output::C);
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

DynamicFunction::WrenchContact::WrenchContact(const mc_rbdyn::RobotFrame & frame, mc_rbdyn::Contact & contact)
: frame_(frame), contact_(&contact), hasVariable_(true), jac_(frame.tvm_frame().rbdJacobian()),
  blocks_(jac_.compactPath(frame.robot().mb())), full_jac_(6, frame.robot().mb().nrDof())
{
  wrench_ = tvm::Space(6).createVariable("wrench " + frame.name());
  wrench_->setZero();
}

DynamicFunction::WrenchContact::WrenchContact(const mc_rbdyn::RobotFrame & frame,
                                              const tvm::VariablePtr & wrench,
                                              mc_rbdyn::Contact & contact)
: frame_(frame), wrench_(wrench), contact_(&contact), hasVariable_(false), jac_(frame.tvm_frame().rbdJacobian()),
  blocks_(jac_.compactPath(frame.robot().mb())), full_jac_(6, frame.robot().mb().nrDof())
{
}

void DynamicFunction::WrenchContact::updateWrenchJacobian(DynamicFunction & parent)
{
  // In surface contact representation, the wrench variable's jacobian is just the contact frame full jac transposed
  const auto & robot = frame_->robot();
  const auto & bodyJac = jac_.bodyJacobian(robot.mb(), robot.mbc());
  full_jac_.setZero();
  jac_.addFullJacobian(blocks_, bodyJac, full_jac_);
  if(hasVariable_)
  {
    parent.jacobian_[wrench_.get()].noalias() = -full_jac_.block(0, 0, 6, robot.mb().nrDof()).transpose();
  }
  else
  {
    // Then this is the second robot with a dynamics function in the contact
    // The jacobian to the wrench var must be transformed from the other frame to this one

    // getting the right transform: if this robot is r1, the var is in frame r2 so we need X_r2_r1
    const auto dualMatrix = contact_->r1Surface()->name() == frame_->name() ? contact_->X_r2s_r1s().dualMatrix()
                                                                            : contact_->X_r2s_r1s().inv().dualMatrix();
    parent.jacobian_[wrench_.get()].noalias() = -full_jac_.block(0, 0, 6, robot.mb().nrDof()).transpose() * dualMatrix;
    // mc_rtc::log::critical("Updating a dual wrench for dynamics constraint");
  }
  // mc_rtc::log::critical("updated wrench jacobian to {}", parent.jacobian_[wrench_.get()]);
}

sva::ForceVecd DynamicFunction::WrenchContact::wrench() const
{
  auto ret = sva::ForceVecd(wrench_->value());
  return ret;
}

const tvm::VariableVector & DynamicFunction::addContact3d(const mc_rbdyn::RobotFrame & frame,
                                                          std::vector<sva::PTransformd> points,
                                                          double dir)
{
  if(frame.robot().name() != robot_.name())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "Attempted to add a contact for {} to dynamic function belonging to {}", frame.robot().name(), robot_.name());
  }
  // Constructs a ForceContact emplaced in contactForces_ (creates force variables tvm::Space(3))
  auto & fc = contactForces_.emplace_back(frame, std::move(points), dir);
  for(const auto & var : fc.forces_) { addVariable(var, true); }
  // Adds dep to call a jacobian update on this function each time the frame jacobian is updated
  addInputDependency<DynamicFunction>(Update::Jacobian, frame.tvm_frame(), mc_tvm::RobotFrame::Output::Jacobian);
  return fc.forces_;
}

const tvm::VariablePtr & DynamicFunction::addContact6d(const mc_rbdyn::RobotFrame & frame, mc_rbdyn::Contact & contact)
{
  if(frame.robot().name() != robot_.name())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "Attempted to add a contact for {} to dynamic function belonging to {}", frame.robot().name(), robot_.name());
  }
  // Constructs a WrenchContact emplaced in contactWrenches_ (creates wrench variable tvm::Space(6))
  auto & wc = contactWrenches_.emplace_back(frame, contact);
  addVariable(wc.wrench_, true);
  // Adds dep to call a jacobian update on this function each time the frame jacobian is updated
  // FIXME Add a dependency on the contact instead of the frame
  addInputDependency<DynamicFunction>(Update::Jacobian, frame.tvm_frame(), mc_tvm::RobotFrame::Output::Jacobian);
  return wc.wrench_;
}

void DynamicFunction::addContact6d(const mc_rbdyn::RobotFrame & frame,
                                   const tvm::VariablePtr & variable,
                                   mc_rbdyn::Contact & contact)
{
  if(frame.robot().name() != robot_.name())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "Attempted to add a contact for {} to dynamic function belonging to {}", frame.robot().name(), robot_.name());
  }
  // Constructs a WrenchContact emplaced in contactWrenches_ and set its variable to the given one
  auto & wc = contactWrenches_.emplace_back(frame, variable, contact);
  addVariable(wc.wrench_, true);
  // Adds dep to call a jacobian update on this function each time the frame jacobian is updated
  // FIXME Add a dependency on the contact instead of the frame, this way if the second frame is updated, this jacobian
  // will be too
  addInputDependency<DynamicFunction>(Update::Jacobian, frame.tvm_frame(), mc_tvm::RobotFrame::Output::Jacobian);
}

void DynamicFunction::removeContact(const mc_rbdyn::RobotFrame & frame)
{
  // Find if this contact corresponds to a 3d or 6d var and remove it
  auto it = findContactForce(frame);
  if(it != contactForces_.end())
  {
    for(const auto & var : it->forces_) { removeVariable(var); }
    contactForces_.erase(it);
  }

  auto it2 = findContactWrench(frame);
  if(it2 != contactWrenches_.end())
  {
    removeVariable(it2->wrench_);
    contactWrenches_.erase(it2);
  }
}

sva::ForceVecd DynamicFunction::contactForce(const mc_rbdyn::RobotFrame & frame) const
{
  auto it = findContactForce(frame);
  auto it2 = findContactWrench(frame);
  if(it != contactForces_.end()) { return (*it).force(); }
  else if(it2 != contactWrenches_.end()) { return (*it2).wrench(); }
  else
  {
    mc_rtc::log::error("No contact at frame {} in dynamic function for {}", frame.name(), robot_.name());
    return sva::ForceVecd(Eigen::Vector6d::Zero());
  }
}

const tvm::VariableVector DynamicFunction::getForceVariables(const std::string & contactFrameName)
{
  tvm::VariableVector variables;
  auto it = findContactForce(robot_.frame(contactFrameName));
  if(it != contactForces_.end())
  {
    // We found this frame in the active contacts for this robot
    variables = (*it).forces_;
  }
  auto it2 = findContactWrench(robot_.frame(contactFrameName));
  if(it2 != contactWrenches_.end())
  {
    // We found this frame in the active contacts for this robot
    variables.add((*it2).wrench_);
  }

  return variables;
}

void DynamicFunction::updateb()
{
  b_ = robot_.tvmRobot().C();
}

void DynamicFunction::updateJacobian()
{
  const auto & robot = robot_.tvmRobot();
  splitJacobian(robot.H(), robot.alphaD());
  // update jacobians for every 3D contact and every 6D contact
  for(auto & c : contactForces_) { c.updateJacobians(*this); }
  for(auto & c : contactWrenches_) { c.updateWrenchJacobian(*this); }
}

auto DynamicFunction::findContactForce(const mc_rbdyn::RobotFrame & frame) const
    -> std::vector<ForceContact>::const_iterator
{
  return std::find_if(contactForces_.begin(), contactForces_.end(),
                      [&](const auto & c) { return c.frame_.get() == &frame; });
}

auto DynamicFunction::findContactWrench(const mc_rbdyn::RobotFrame & frame) const
    -> std::vector<WrenchContact>::const_iterator
{
  return std::find_if(contactWrenches_.begin(), contactWrenches_.end(),
                      [&](const auto & c) { return c.frame_.get() == &frame; });
}

} // namespace mc_tvm
