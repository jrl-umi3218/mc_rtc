/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/OrientationFunction.h>

#include <mc_tvm/Robot.h>
#include <mc_tvm/RobotFrame.h>

namespace mc_tvm
{

OrientationFunction::OrientationFunction(const mc_rbdyn::RobotFrame & frame)
: tvm::function::abstract::Function(3), robot_frame_(frame), frame_(frame.tvm_frame())
{
  reset();
  // clang-format off
  registerUpdates(Update::Value, &OrientationFunction::updateValue,
                  Update::Velocity, &OrientationFunction::updateVelocity,
                  Update::Jacobian, &OrientationFunction::updateJacobian,
                  Update::NormalAcceleration, &OrientationFunction::updateNormalAcceleration);
  // clang-format on
  addOutputDependency<OrientationFunction>(Output::Value, Update::Value);
  addOutputDependency<OrientationFunction>(Output::Velocity, Update::Velocity);
  addOutputDependency<OrientationFunction>(Output::Jacobian, Update::Jacobian);
  addOutputDependency<OrientationFunction>(Output::NormalAcceleration, Update::NormalAcceleration);
  const auto & robot = robot_frame_->robot().tvmRobot();
  addVariable(robot.q(), false);
  addInputDependency<OrientationFunction>(Update::Value, frame_, mc_tvm::RobotFrame::Output::Position);
  addInputDependency<OrientationFunction>(Update::Velocity, frame_, mc_tvm::RobotFrame::Output::Velocity);
  addInputDependency<OrientationFunction>(Update::Jacobian, frame_, mc_tvm::RobotFrame::Output::Jacobian);
  addInputDependency<OrientationFunction>(Update::NormalAcceleration, frame_,
                                          mc_tvm::RobotFrame::Output::NormalAcceleration);
}

void OrientationFunction::reset()
{
  ori_ = frame_.position().rotation();
  refVel_.setZero();
  refAccel_.setZero();
}

void OrientationFunction::updateValue()
{
  value_ = sva::rotationError(ori_, frame_.position().rotation());
}

void OrientationFunction::updateVelocity()
{
  velocity_ = frame_.velocity().angular() - refVel_;
}

void OrientationFunction::updateJacobian()
{
  const auto & robot = robot_frame_->robot().tvmRobot();
  splitJacobian(frame_.jacobian().topRows<3>(), robot.q());
}

void OrientationFunction::updateNormalAcceleration()
{
  normalAcceleration_ = frame_.normalAcceleration().angular() - refAccel_;
}

} // namespace mc_tvm
