/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tvm/api.h>
#include <mc_tvm/fwd.h>

#include <mc_rbdyn/fwd.h>

#include <tvm/function/abstract/Function.h>

#include <RBDyn/Jacobian.h>

#include <optional>

namespace mc_tvm
{

/** This class implements a gaze function for a given camera frame
 *
 * This is the difference between a 2d point in the image and a reference as a
 * function of the robot configuration.
 *
 * This task value is to be updated by an external process (e.g. a vision algorithm)
 *
 * The function does not make sense if the camera points are not provided in the control frame
 */
struct MC_TVM_DLLAPI GazeFunction : tvm::function::abstract::Function
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SET_UPDATES(GazeFunction, Value, Velocity, Jacobian, NormalAcceleration)

  /** Constructor
   *
   * \param frame Camera frame
   */
  GazeFunction(const mc_rbdyn::RobotFrame & frame);

  /** Reset the objective to the current position */
  inline void reset()
  {
    pointRef_ = point_;
  }

  /** Set the current point value
   *
   * \throws If \param depth <= 0
   */
  void estimate(const Eigen::Vector2d & point, std::optional<double> depth = std::nullopt);

  /** Set the current point value
   *
   * \throws if \param point.z() <= 0
   */
  void estimate(const Eigen::Vector3d & point);

  /** Set the desired value */
  inline void target(const Eigen::Vector2d & ref) noexcept
  {
    pointRef_ = ref;
  }

  /** Get the desired value */
  inline const Eigen::Vector2d & target() const noexcept
  {
    return pointRef_;
  }

  /** Access the controlled frame */
  inline const mc_rbdyn::RobotFrame & frame() const noexcept
  {
    return *frame_;
  }

protected:
  mc_rbdyn::ConstRobotFramePtr frame_;
  mc_tvm::RobotFrame & tvm_frame_;
  rbd::Jacobian frameJac_;

  /* Inputs to the function */
  Eigen::Vector2d point_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d pointRef_ = Eigen::Vector2d::Zero();
  double depthEstimate_ = 1.0;

  /* Computation intermediates */
  Eigen::Matrix<double, 2, 6> L_img_;
  Eigen::Matrix<double, 6, 1> surfaceVelocity_;
  Eigen::Matrix<double, 1, 6> L_Z_dot_;
  Eigen::Matrix<double, 2, 6> L_img_dot_;

  /* TVM update methods */
  void updateValue();
  void updateVelocity();
  void updateJacobian();
  Eigen::MatrixXd shortJacMat_;
  Eigen::MatrixXd jacMat_;
  void updateNormalAcceleration();

  void setDepthEstimate(double depth);
};

} // namespace mc_tvm
