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
  /* Note: in this original ForceContact formulation the jacobian chosen for the force is the bodyJacobian
  This means the force variables are chosen to be in body frame, and they are translated for each point in body frame.
  */
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
  const auto & robot = frame_->robot();

  /* IMPORTANT: in the original ForceContact formulation the jacobian chosen for the force is the bodyJacobian
  This means the force variables are chosen to be in body frame, and they are translated for each point in body frame.
  Here we would like the variables in contact frame so that they can be directly reused easily (for example
  in the CoMWrenchTransforms_) so we use the jacobian at the contact frame instead.
  */

  const auto X_0_contact = frame_->position();
  const auto & contactJac = jac_.jacobian(robot.mb(), robot.mbc(), X_0_contact);
  full_jac_.setZero();
  // In surface contact representation, the wrench variable's jacobian is just the contact frame full jac transposed
  jac_.addFullJacobian(blocks_, contactJac, full_jac_);

  if(hasVariable_)
  {
    parent.jacobian_[wrench_.get()].noalias() = -full_jac_.block(0, 0, 6, robot.mb().nrDof()).transpose();
    // Update transform map
    // FIXME make sure the frame AND the com are up to date and use the tvm versions
    auto X_0_f = frame_->position();
    auto X_0_C = sva::PTransformd(frame_->robot().com());
    auto X_f_C = (X_0_C * X_0_f.inv());
    parent.CoMWrenchTransforms_[wrench_] = X_f_C;
  }
  else
  {
    // We don't own the var so this is the second robot with a dynamics function in the contact.
    // The jacobian to the wrench var must be transformed from the other frame to this one, and negated
    // This way the wrench applied in the second contact frame results in - the first wrench if expressed in the
    // first contact frame

    // Getting the right transform: if this robot is r1, the var is in frame r2 so we need X_r2_r1
    const auto X_var_f =
        contact_->r1Surface()->name() == frame_->name() ? contact_->X_r2s_r1s() : contact_->X_r2s_r1s().inv();

    parent.jacobian_[wrench_.get()].noalias() =
        -full_jac_.block(0, 0, 6, robot.mb().nrDof()).transpose() * -X_var_f.dualMatrix();
    // Then we simply emplace the transform between the variable frame and this robot's CoM,
    // taking the opposite into account
    // FIXME make sure the frame AND the com are up to date and use the tvm versions
    auto X_0_f = frame_->position();
    auto X_0_C = sva::PTransformd(frame_->robot().com());
    // This transforms to the negative frame
    auto X_negative = sva::PTransformd(-Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

    auto X_var_C = X_negative * X_0_C * X_0_f.inv() * X_var_f;
    parent.CoMWrenchTransforms_[wrench_] = X_var_C;
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
  // add this variable to the dual wrenches map (it was created on this side)
  auto X_0_f = frame.position();
  auto X_0_C = sva::PTransformd(robot_.tvmRobot().comAlgo().com());
  auto X_f_C = (X_0_C * X_0_f.inv());
  CoMWrenchTransforms_.emplace(wc.wrench_, X_f_C);
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
  // add this variable to the dual wrenches map (it was created by the other frame of the contact)
  auto X_0_f = frame.position();
  auto X_0_C = sva::PTransformd(robot_.tvmRobot().comAlgo().com());
  // Getting the transform from variable to this robot's frame: if this robot is r1,
  // the var is in frame r2 so we need X_r2_r1, otherwise X_r1_r2
  // After this, since the given variable is opposite to this one (Newton) we negate the wrench value in the transform
  auto X_negative = sva::PTransformd(-Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
  const auto X_var_f = contact.r1Surface()->name() == frame.name() ? contact.X_r2s_r1s() : contact.X_r2s_r1s().inv();
  auto X_var_C = X_0_C * X_0_f.inv() * X_negative * X_var_f;
  CoMWrenchTransforms_.emplace(wc.wrench_, X_var_C);
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
    CoMWrenchTransforms_.erase(it2->wrench_);
    removeVariable(it2->wrench_);
    contactWrenches_.erase(it2);
  }
}

sva::ForceVecd DynamicFunction::contactForce(const mc_rbdyn::RobotFrame & frame) const
{
  auto it = findContactForce(frame);
  auto it2 = findContactWrench(frame);
  if(it != contactForces_.end()) { return (*it).force(); }
  else if(it2 != contactWrenches_.end())
  {
    // Check if need to return the wrench var as is or transform var (other side)
    if(it2->hasVariable_) { return (*it2).wrench(); }
    else
    {
      // getting the right transform: if this robot is r1, the var is in frame r2 so we need X_r2_r1
      // if this is r2, variable is w1 so we need X_r1_r2
      const auto dualMatrix = it2->contact_->r1Surface()->name() == it2->frame_->name()
                                  ? it2->contact_->X_r2s_r1s().dualMatrix()
                                  : it2->contact_->X_r2s_r1s().inv().dualMatrix();
      return sva::ForceVecd(-dualMatrix * it2->wrench().vector());
    }
  }
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
