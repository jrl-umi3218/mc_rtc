/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/FrameVelocity.h>

#include <mc_tvm/Robot.h>
#include <mc_tvm/RobotFrame.h>

#include <mc_rbdyn/Robot.h>

namespace mc_tvm
{

FrameVelocity::FrameVelocity(const mc_rbdyn::RobotFrame & frame, const Eigen::Vector6d & dof)
: tvm::function::abstract::Function(6), frame_(frame), dof_(dof),
  jac_(rbd::Jacobian(frame.robot().mb(), frame.body(), frame.X_b_f().translation())),
  blocks_(jac_.compactPath(frame.robot().mb())), jacobian_(Eigen::MatrixXd::Zero(6, frame.robot().mb().nrDof()))
{
  // clang-format off
  registerUpdates(Update::Value, &FrameVelocity::updateValue,
                  Update::Velocity, &FrameVelocity::updateVelocity,
                  Update::Jacobian, &FrameVelocity::updateJacobian);
  // clang-format on
  addOutputDependency<FrameVelocity>(Output::Value, Update::Value);
  addOutputDependency<FrameVelocity>(Output::Velocity, Update::Velocity);
  addOutputDependency<FrameVelocity>(Output::Jacobian, Update::Jacobian);
  auto & tvm_robot = frame.robot().tvmRobot();
  addVariable(tvm::dot(tvm_robot.q()), false);
  addInputDependency<FrameVelocity>(Update::Value, tvm_robot, mc_tvm::Robot::Output::FV);
  addInputDependency<FrameVelocity>(Update::Jacobian, tvm_robot, mc_tvm::Robot::Output::FV);
  addInputDependency<FrameVelocity>(Update::Velocity, tvm_robot, mc_tvm::Robot::Output::NormalAcceleration);

  // Make all values up-to-date on creation
  updateValue();
  updateVelocity();
  updateJacobian();
}

void FrameVelocity::updateValue()
{
  value_ = dof_.cwiseProduct((frame_->X_b_f() * jac_.bodyVelocity(frame_->robot().mb(), frame_->robot().mbc())).vector()
                             - refVel_);
}

void FrameVelocity::updateJacobian()
{
  const auto & jac = jac_.bodyJacobian(frame_->robot().mb(), frame_->robot().mbc());
  jacobian_.setZero();
  jac_.addFullJacobian(blocks_, dof_.asDiagonal() * frame_->X_b_f().matrix() * jac, jacobian_);
  splitJacobian(jacobian_, tvm::dot(frame_->robot().tvmRobot().q()));
}

void FrameVelocity::updateVelocity()
{
  velocity_ = dof_.cwiseProduct((frame_->X_b_f()
                                 * jac_.bodyNormalAcceleration(frame_->robot().mb(), frame_->robot().mbc(),
                                                               frame_->robot().tvmRobot().normalAccB()))
                                    .vector()
                                - refAccel_);
}

} // namespace mc_tvm
