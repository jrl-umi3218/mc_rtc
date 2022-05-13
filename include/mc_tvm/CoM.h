/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tvm/api.h>
#include <mc_tvm/fwd.h>

#include <mc_rtc/shared.h>

#include <RBDyn/CoM.h>

#include <tvm/graph/abstract/Node.h>

namespace mc_tvm
{

/** Center of mass (CoM) of a Robot and related quantities
 *
 * Provides the frame position, jacobian, velocity and normal acceleration.
 * These signals are correctly initialized on the object's creation.
 *
 * Outputs:
 * - Position: position of the CoM in world coordinates
 * - Jacobian: jacobian of the CoM in world coordinates
 * - Velocity: velocity of the CoM in world coordinates
 * - NormalAcceleration: normal acceleration of the CoM in world coordinates
 * - Acceleration: acceleration of the CoM in world coordinates
 * - JDot: derivative of the jacobian of the CoM in world coordinates
 *
 */
struct MC_TVM_DLLAPI CoM : public tvm::graph::abstract::Node<CoM>
{
  SET_OUTPUTS(CoM, CoM, Jacobian, Velocity, NormalAcceleration, Acceleration, JDot)
  SET_UPDATES(CoM, CoM, Jacobian, Velocity, NormalAcceleration, Acceleration, JDot)

  friend struct Robot;

private:
  struct NewCoMToken
  {
  };

public:
  /** Constructor
   *
   * Creates the CoM algorithm for a robot
   *
   * \param robot Robot to which the frame is attached
   *
   */
  CoM(NewCoMToken, Robot & robot);

  inline const Eigen::Vector3d & com() const noexcept
  {
    return com_;
  }

  inline const Eigen::Vector3d & velocity() const noexcept
  {
    return velocity_;
  }

  inline const Eigen::Vector3d & normalAcceleration() const noexcept
  {
    return normalAcceleration_;
  }

  inline const Eigen::Vector3d & acceleration() const noexcept
  {
    return acceleration_;
  }

  inline const Eigen::MatrixXd & jacobian() const noexcept
  {
    return jac_.jacobian();
  }

  inline const Eigen::MatrixXd & JDot() const noexcept
  {
    return jac_.jacobianDot();
  }

  inline const Robot & robot() const noexcept
  {
    return robot_;
  }

  inline Robot & robot() noexcept
  {
    return robot_;
  }

  /** Access the underlying CoMJacobian object to perform computations */
  inline const rbd::CoMJacobian & comJacobian() const noexcept
  {
    return jac_;
  }

private:
  Robot & robot_;
  rbd::CoMJacobian jac_;

  Eigen::Vector3d com_;
  void updateCoM();

  Eigen::Vector3d velocity_;
  void updateVelocity();

  Eigen::Vector3d normalAcceleration_;
  void updateNormalAcceleration();

  Eigen::Vector3d acceleration_;
  void updateAcceleration();

  void updateJacobian();

  void updateJDot();
};

} // namespace mc_tvm
