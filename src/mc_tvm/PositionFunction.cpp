/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/PositionFunction.h>

#include <mc_tvm/Robot.h>
#include <mc_tvm/RobotFrame.h>

namespace mc_tvm
{

PositionFunction::PositionFunction(const mc_rbdyn::RobotFrame & frame)
: tvm::function::abstract::Function(3), robot_frame_(frame), frame_(frame.tvm_frame())
{
  reset();
  // clang-format off
  registerUpdates(Update::Value, &PositionFunction::updateValue,
                  Update::Velocity, &PositionFunction::updateVelocity,
                  Update::Jacobian, &PositionFunction::updateJacobian,
                  Update::NormalAcceleration, &PositionFunction::updateNormalAcceleration);
  // clang-format on
  addOutputDependency<PositionFunction>(Output::Value, Update::Value);
  addOutputDependency<PositionFunction>(Output::Velocity, Update::Velocity);
  addOutputDependency<PositionFunction>(Output::Jacobian, Update::Jacobian);
  addOutputDependency<PositionFunction>(Output::NormalAcceleration, Update::NormalAcceleration);
  auto & robot = robot_frame_->robot().tvmRobot();
  addVariable(robot.q(), false);
  addInputDependency<PositionFunction>(Update::Value, frame_, mc_tvm::RobotFrame::Output::Position);
  addInputDependency<PositionFunction>(Update::Velocity, frame_, mc_tvm::RobotFrame::Output::Velocity);
  addInputDependency<PositionFunction>(Update::Jacobian, frame_, mc_tvm::RobotFrame::Output::Jacobian);
  addInputDependency<PositionFunction>(Update::NormalAcceleration, frame_,
                                       mc_tvm::RobotFrame::Output::NormalAcceleration);
}

void PositionFunction::reset()
{
  pos_ = frame_.position().translation();
  refVel_.setZero();
  refAccel_.setZero();
}

void PositionFunction::updateValue()
{
  value_ = frame_.position().translation() - pos_;
}

void PositionFunction::updateVelocity()
{
  velocity_ = frame_.velocity().linear() - refVel_;
}

void PositionFunction::updateJacobian()
{
  splitJacobian(frame_.jacobian().bottomRows<3>(), robot_frame_->robot().tvmRobot().q());
}

void PositionFunction::updateNormalAcceleration()
{
  normalAcceleration_ = frame_.normalAcceleration().linear() - refAccel_;
}

} // namespace mc_tvm
