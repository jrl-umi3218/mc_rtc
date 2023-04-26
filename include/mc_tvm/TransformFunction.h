/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tvm/api.h>
#include <mc_tvm/fwd.h>

#include <mc_rbdyn/fwd.h>

#include <tvm/function/abstract/Function.h>

#include <RBDyn/Jacobian.h>

#include <SpaceVecAlg/SpaceVecAlg>

namespace mc_tvm
{

/** This class implements a position function for a given frame */
class MC_TVM_DLLAPI TransformFunction : public tvm::function::abstract::Function
{
public:
  SET_UPDATES(TransformFunction, Value, Velocity, Jacobian, NormalAcceleration)

  /** Constructor
   *
   * Set the objective to the current frame pose
   *
   */
  TransformFunction(const mc_rbdyn::RobotFrame & frame);

  /** Set the target pose to the current frame pose */
  void reset();

  /** Get the current objective */
  inline const sva::PTransformd & pose() const noexcept { return pose_; }

  /** Set the objective */
  inline void pose(const sva::PTransformd & pose) noexcept { pose_ = pose; }

  /** Get the current objective */
  inline const Eigen::Vector6d & refVel() const noexcept { return refVel_; }

  /** Set the objective */
  inline void refVel(const Eigen::Vector6d & refVel) noexcept { refVel_ = refVel; }

  /** Get the current objective */
  inline const Eigen::Vector6d & refAccel() const noexcept { return refAccel_; }

  /** Set the objective */
  inline void refAccel(const Eigen::Vector6d & refAccel) noexcept { refAccel_ = refAccel; }

  /** Get the frame */
  inline const mc_rbdyn::RobotFrame & frame() const noexcept { return *frame_; }

protected:
  void updateValue();
  void updateVelocity();
  void updateJacobian();
  void updateNormalAcceleration();

  mc_rbdyn::ConstRobotFramePtr frame_;
  mc_tvm::RobotFrame & tvm_frame_;

  /** Computation intermediate */
  rbd::Jacobian frameJac_;
  Eigen::MatrixXd shortJacMat_;
  Eigen::MatrixXd jacMat_;
  sva::MotionVecd err_p_;
  sva::MotionVecd w_p_p_;
  sva::MotionVecd V_err_p_;

  /** Target */
  sva::PTransformd pose_;
  Eigen::Vector6d refVel_;
  Eigen::Vector6d refAccel_;
};

} // namespace mc_tvm
