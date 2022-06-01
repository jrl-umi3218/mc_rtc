/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/PositionBasedVisServoFunction.h>

#include <mc_tvm/Robot.h>
#include <mc_tvm/RobotFrame.h>

#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/RobotFrame.h>

#include <RBDyn/VisServo.h>

namespace mc_tvm
{

PositionBasedVisServoFunction::PositionBasedVisServoFunction(const mc_rbdyn::RobotFrame & frame)
: tvm::function::abstract::Function(6), frame_(frame), tvm_frame_(frame.tvm_frame()),
  frameJac_(tvm_frame_.rbdJacobian()), shortJacMat_(6, frameJac_.dof()), jacMat_(6, frame.robot().mb().nrDof())
{
  L_pbvs_dot_.setZero();
  // clang-format off
  registerUpdates(Update::Value,    &PositionBasedVisServoFunction::updateValue,
                  Update::Velocity, &PositionBasedVisServoFunction::updateVelocity,
                  Update::Jacobian, &PositionBasedVisServoFunction::updateJacobian,
                  Update::NormalAcceleration, &PositionBasedVisServoFunction::updateNormalAcceleration);
  // clang-format on
  addOutputDependency<PositionBasedVisServoFunction>(Output::Value, Update::Value);
  addOutputDependency<PositionBasedVisServoFunction>(Output::Velocity, Update::Velocity);
  addOutputDependency<PositionBasedVisServoFunction>(Output::Jacobian, Update::Jacobian);
  addOutputDependency<PositionBasedVisServoFunction>(Output::NormalAcceleration, Update::NormalAcceleration);
  const auto & robot = frame_->robot();
  const auto & tvm_robot = robot.tvmRobot();
  addVariable(tvm_robot.q(), false);
  addInputDependency<PositionBasedVisServoFunction>(Update::Value, tvm_frame_, mc_tvm::RobotFrame::Output::Position);
  addInputDependency<PositionBasedVisServoFunction>(Update::Velocity, tvm_frame_, mc_tvm::RobotFrame::Output::Velocity);
  addInputDependency<PositionBasedVisServoFunction>(Update::Jacobian, tvm_frame_, mc_tvm::RobotFrame::Output::Jacobian);
  addInputDependency<PositionBasedVisServoFunction>(Update::NormalAcceleration, tvm_frame_,
                                                    mc_tvm::RobotFrame::Output::NormalAcceleration);
  // Jacobian needs L_pbvs_
  addInternalDependency<PositionBasedVisServoFunction>(Update::Jacobian, Update::Velocity);
  // NA needs L_pbvs_ and velocity_
  addInternalDependency<PositionBasedVisServoFunction>(Update::NormalAcceleration, Update::Velocity);
}

void PositionBasedVisServoFunction::updateValue()
{
  rbd::getAngleAxis(X_t_s_.rotation().transpose(), angle_, axis_);
  value_.head(3) = -angle_ * axis_;
  value_.tail(3) = X_t_s_.translation();
}

void PositionBasedVisServoFunction::updateVelocity()
{
  rbd::poseJacobian(X_t_s_.rotation(), L_pbvs_);
  const auto & robot = frame_->robot();
  frameVelocity_ = frameJac_.velocity(robot.mb(), robot.mbc(), frame_->X_b_f()).vector();
  velocity_.noalias() = L_pbvs_ * frameVelocity_;
}

void PositionBasedVisServoFunction::updateJacobian()
{
  const auto & robot = frame_->robot();
  auto & tvm_robot = robot.tvmRobot();
  shortJacMat_.noalias() = L_pbvs_ * frameJac_.jacobian(robot.mb(), robot.mbc(), tvm_frame_.position());
  frameJac_.fullJacobian(robot.mb(), shortJacMat_, jacMat_);
  splitJacobian(jacMat_, tvm_robot.q());
}

void PositionBasedVisServoFunction::updateNormalAcceleration()
{
  const auto & robot = frame_->robot();
  auto & tvm_robot = robot.tvmRobot();
  rbd::getSkewSym(frameVelocity_.head(3), omegaSkew_);
  L_pbvs_dot_.block(3, 3, 3, 3) << -X_t_s_.rotation().transpose() * omegaSkew_;
  normalAcceleration_.noalias() = L_pbvs_
                                      * frameJac_
                                            .normalAcceleration(robot.mb(), robot.mbc(), tvm_robot.normalAccB(),
                                                                frame_->X_b_f(), sva::MotionVecd::Zero())
                                            .vector()
                                  + L_pbvs_dot_ * frameVelocity_;
}

} // namespace mc_tvm
