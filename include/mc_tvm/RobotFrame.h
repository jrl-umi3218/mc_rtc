/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tvm/Frame.h>

#include <mc_rbdyn/RobotFrame.h>

namespace mc_tvm
{

/** A frame associated to an \ref mc_rbdyn::RobotFrame
 *
 * Provides the frame position, jacobian, velocity and normal acceleration.
 * These signals are correctly initialized on the object's creation.
 *
 * Outputs:
 * - Position: position of the frame in world coordinates
 * - Jacobian: jacobian of the frame in world coordinates
 * - Velocity: velocity of the frame in world coordinates
 * - NormalAcceleration: normal acceleration of the frame in world coordinates
 * - JDot: derivative of the jacobian of the frame in world coordinate
 *
 */
struct MC_TVM_DLLAPI RobotFrame : public Frame
{
  SET_OUTPUTS(RobotFrame, Jacobian, NormalAcceleration, JDot)
  SET_UPDATES(RobotFrame, Jacobian, NormalAcceleration, JDot)

  friend struct mc_rbdyn::RobotFrame;

protected:
  struct NewRobotFrameToken : public Frame::NewFrameToken
  {
  };

public:
  RobotFrame(NewRobotFrameToken, const mc_rbdyn::RobotFrame & frame);

  /** Frame's jacobian */
  inline const tvm::internal::MatrixWithProperties & jacobian() const noexcept
  {
    return jacobian_;
  }

  /** Frame's normal acceleration in inertial frame */
  inline const sva::MotionVecd & normalAcceleration() const noexcept
  {
    return normalAcceleration_;
  }

  /** Frame's jacobian time derivative */
  inline const tvm::internal::MatrixWithProperties & JDot() const noexcept
  {
    return jacDot_;
  }

  /** Access the internal RBDyn Jacobian object
   *
   * You can copy the jacobian from this reference.
   *
   * This is useful if your entity requires:
   * - the body jacobian
   * - the jacobian translated at varying position
   */
  inline const rbd::Jacobian & rbdJacobian() const noexcept
  {
    return jac_;
  }

  /** Returns the associated mc_rbdyn RobotFrame */
  inline const mc_rbdyn::RobotFrame & frame() const noexcept
  {
    return static_cast<const mc_rbdyn::RobotFrame &>(frame_);
  }

protected:
  Eigen::Matrix3d h_ = Eigen::Matrix3d::Zero();

  rbd::Jacobian jac_;
  rbd::Blocks blocks_;

  Eigen::MatrixXd jacTmp_;

  tvm::internal::MatrixWithProperties jacobian_;
  void updateJacobian();

  sva::MotionVecd normalAcceleration_;
  void updateNormalAcceleration();

  tvm::internal::MatrixWithProperties jacDot_;
  void updateJDot();
};

} // namespace mc_tvm
