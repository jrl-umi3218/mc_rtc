/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/GazeFunction.h>

#include <mc_tvm/Robot.h>
#include <mc_tvm/RobotFrame.h>

#include <RBDyn/VisServo.h>

namespace mc_tvm
{

GazeFunction::GazeFunction(const mc_rbdyn::RobotFrame & frame)
: tvm::function::abstract::Function(2), frame_(frame), tvm_frame_(frame.tvm_frame()),
  frameJac_(tvm_frame_.rbdJacobian()), shortJacMat_(2, frameJac_.dof()), jacMat_(2, frame.robot().mb().nrDof())
{
  // clang-format off
  registerUpdates(Update::Value,    &GazeFunction::updateValue,
                  Update::Velocity, &GazeFunction::updateVelocity,
                  Update::Jacobian, &GazeFunction::updateJacobian,
                  Update::NormalAcceleration, &GazeFunction::updateNormalAcceleration);
  // clang-format on
  addOutputDependency<GazeFunction>(Output::Value, Update::Value);
  addOutputDependency<GazeFunction>(Output::Velocity, Update::Velocity);
  addOutputDependency<GazeFunction>(Output::Jacobian, Update::Jacobian);
  addOutputDependency<GazeFunction>(Output::NormalAcceleration, Update::NormalAcceleration);
  const auto & robot = frame_->robot();
  const auto & tvm_robot = robot.tvmRobot();
  addVariable(tvm_robot.q(), false);
  addInputDependency<GazeFunction>(Update::Value, tvm_frame_, mc_tvm::RobotFrame::Output::Position);
  addInputDependency<GazeFunction>(Update::Velocity, tvm_frame_, mc_tvm::RobotFrame::Output::Velocity);
  addInputDependency<GazeFunction>(Update::Jacobian, tvm_frame_, mc_tvm::RobotFrame::Output::Jacobian);
  addInputDependency<GazeFunction>(Update::NormalAcceleration, tvm_frame_,
                                   mc_tvm::RobotFrame::Output::NormalAcceleration);
  // Jacobian needs L_img_
  addInternalDependency<GazeFunction>(Update::Jacobian, Update::Velocity);
  // NA needs L_img_ and velocity_
  addInternalDependency<GazeFunction>(Update::NormalAcceleration, Update::Velocity);
}

void GazeFunction::estimate(const Eigen::Vector2d & point, std::optional<double> depth)
{
  point_ = point;
  if(depth) { setDepthEstimate(depth.value()); }
}

void GazeFunction::estimate(const Eigen::Vector3d & point)
{
  point_ = point.head(2);
  setDepthEstimate(point(2));
}

void GazeFunction::setDepthEstimate(double d)
{
  depthEstimate_ = d;
  if(depthEstimate_ <= 0)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[GazeFunction] depth estimate must be strictly positive");
  }
}

void GazeFunction::updateValue()
{
  value_ = point_ - pointRef_;
}

void GazeFunction::updateVelocity()
{
  const auto & robot = frame_->robot();
  rbd::imagePointJacobian(point_, depthEstimate_, L_img_);
  surfaceVelocity_ = frameJac_.velocity(robot.mb(), robot.mbc(), frame_->X_b_f()).vector();
  velocity_ = L_img_ * surfaceVelocity_;
}

void GazeFunction::updateJacobian()
{
  const auto & robot = frame_->robot();
  shortJacMat_ = L_img_ * frameJac_.jacobian(robot.mb(), robot.mbc(), tvm_frame_.position());
  frameJac_.fullJacobian(robot.mb(), shortJacMat_, jacMat_);
  splitJacobian(jacMat_, frame_->robot().tvmRobot().q());
}

void GazeFunction::updateNormalAcceleration()
{
  const auto & robot = frame_->robot();
  rbd::depthDotJacobian(velocity_, depthEstimate_, L_Z_dot_);
  rbd::imagePointJacobianDot(point_, velocity_, depthEstimate_, L_Z_dot_ * surfaceVelocity_, L_img_dot_);
  normalAcceleration_ = L_img_
                            * frameJac_
                                  .normalAcceleration(robot.mb(), robot.mbc(), robot.tvmRobot().normalAccB(),
                                                      frame_->X_b_f(), sva::MotionVecd::Zero())
                                  .vector()
                        + L_img_dot_ * surfaceVelocity_;
}

} // namespace mc_tvm
