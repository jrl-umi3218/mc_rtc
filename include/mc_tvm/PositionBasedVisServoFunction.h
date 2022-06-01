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

/** This class implements a position-based visual servoing function
 *
 * This task value is to be updated by an external process (e.g. a vision algorithm)
 *
 * See https://hal.inria.fr/hal-01421734/file/2016_ral_agaravante.pdf for details
 *
 */
struct MC_TVM_DLLAPI PositionBasedVisServoFunction : tvm::function::abstract::Function
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SET_UPDATES(PositionBasedVisServoFunction, Value, Velocity, Jacobian, NormalAcceleration)

  /** Constructor
   *
   * \param frame Controlled frame
   */
  PositionBasedVisServoFunction(const mc_rbdyn::RobotFrame & frame);

  /** Reset the error */
  inline void reset() noexcept
  {
    X_t_s_ = sva::PTransformd::Identity();
  }

  /** Access the current error */
  inline const sva::PTransformd & error() const noexcept
  {
    return X_t_s_;
  }

  /** Set the function error, i.e. the current position relative to the target position */
  inline void error(const sva::PTransformd & X_t_s) noexcept
  {
    X_t_s_ = X_t_s;
  }

  /** Access the controlled frame */
  inline const mc_rbdyn::RobotFrame & frame() const noexcept
  {
    return *frame_;
  }

protected:
  mc_rbdyn::ConstRobotFramePtr frame_;
  mc_tvm::RobotFrame & tvm_frame_;

  /* Inputs to the function */
  sva::PTransformd X_t_s_;

  /* Computation intermediates */
  rbd::Jacobian frameJac_;
  double angle_;
  Eigen::Vector3d axis_;
  Eigen::Matrix<double, 6, 6> L_pbvs_;
  Eigen::Matrix<double, 6, 1> frameVelocity_;
  Eigen::Matrix3d omegaSkew_;
  Eigen::Matrix<double, 6, 6> L_pbvs_dot_;

  /* TVM update methods */
  void updateValue();
  void updateVelocity();
  void updateJacobian();
  Eigen::MatrixXd shortJacMat_;
  Eigen::MatrixXd jacMat_;
  void updateNormalAcceleration();
};

} // namespace mc_tvm
