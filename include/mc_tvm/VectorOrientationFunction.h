/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tvm/api.h>
#include <mc_tvm/fwd.h>

#include <mc_rbdyn/fwd.h>

#include <tvm/function/abstract/Function.h>

#include <RBDyn/Jacobian.h>

namespace mc_tvm
{

/** For a given vector attached to a frame expressed in frame coordinates, computes the difference with a target vector
 * in world coordinates*/
class MC_TVM_DLLAPI VectorOrientationFunction : public tvm::function::abstract::Function
{
public:
  SET_UPDATES(VectorOrientationFunction, Value, Velocity, Jacobian, NormalAcceleration)

  /** Constructor
   *
   * Set the objective to the current value of the body vector in world coordinates
   *
   */
  VectorOrientationFunction(const mc_rbdyn::RobotFrame & frame, const Eigen::Vector3d & frameVector);

  /** Reset the target vector to the current frame vector */
  void reset();

  /** Get the current objective */
  inline const Eigen::Vector3d & target() const noexcept
  {
    return target_;
  }

  /** Set the objective */
  inline void target(const Eigen::Vector3d & target) noexcept
  {
    target_ = target;
  }

  /** Get the current frame vector (in frame coordinates) */
  inline const Eigen::Vector3d & frameVector() const noexcept
  {
    return frameVectorIn_;
  }

  /** Set the frame vector */
  void frameVector(const Eigen::Vector3d & frameVector) noexcept;

  /** Returns the current value of frame vector in world coordinates */
  inline const Eigen::Vector3d & actual() const noexcept
  {
    return actualVector_;
  }

  /** Get the current objective */
  inline const Eigen::Vector3d & refVel() const noexcept
  {
    return refVel_;
  }

  /** Set the objective */
  inline void refVel(const Eigen::Vector3d & refVel) noexcept
  {
    refVel_ = refVel;
  }

  /** Get the current objective */
  inline const Eigen::Vector3d & refAccel() const noexcept
  {
    return refAccel_;
  }

  /** Set the objective */
  inline void refAccel(const Eigen::Vector3d & refAccel) noexcept
  {
    refAccel_ = refAccel;
  }

  /** Access the frame */
  inline const mc_rbdyn::RobotFrame & frame() const noexcept
  {
    return *frame_;
  }

protected:
  void updateValue();
  void updateVelocity();
  void updateJacobian();
  void updateNormalAcceleration();

  mc_rbdyn::ConstRobotFramePtr frame_;
  mc_tvm::RobotFrame & body_frame_;

  /** Task configuration */
  Eigen::Vector3d frameVectorIn_;

  /** Computation intermediate */
  rbd::Jacobian jac_;
  Eigen::Vector3d actualVector_;
  Eigen::Vector3d bodyVector_;
  Eigen::Matrix3d bodyVectorHat_;
  Eigen::MatrixXd fullJacobian_;
  Eigen::Matrix3d E_0_b_;
  Eigen::Vector3d w_b_b_;

  /** Target */
  Eigen::Vector3d target_;
  Eigen::Vector3d refVel_;
  Eigen::Vector3d refAccel_;
};

} // namespace mc_tvm
