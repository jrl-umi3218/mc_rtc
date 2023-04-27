/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tvm/CoM.h>

#include <RBDyn/Momentum.h>

namespace mc_tvm
{

/** Momentum of a Robot and related quantities
 *
 * Provides the momentum, jacobian, velocity and normal acceleration.
 * These signals are correctly initialized on the object's creation.
 *
 * Outputs:
 * - Position: position of the momentum in world coordinates
 * - Jacobian: jacobian of the momentum in world coordinates
 * - NormalAcceleration: normal acceleration of the momentum in world coordinates
 * - JDot: derivative of the jacobian of the momentum in world coordinates
 *
 */
struct MC_TVM_DLLAPI Momentum : public tvm::graph::abstract::Node<Momentum>
{
  SET_OUTPUTS(Momentum, Momentum, Jacobian, Velocity, NormalAcceleration, JDot)
  SET_UPDATES(Momentum, Momentum, Jacobian, NormalAcceleration, JDot)

  friend struct Robot;

private:
  struct NewMomentumToken
  {
  };

public:
  /** Constructor
   *
   * Creates the momentum algorithm for a robot
   *
   * \param robot Robot to which the frame is attached
   *
   */
  Momentum(NewMomentumToken, CoM & com);

  inline const auto & momentum() const noexcept { return momentum_; }

  inline const auto & velocity() const noexcept { return velocity_; }

  inline const auto & normalAcceleration() const noexcept { return normalAcceleration_; }

  inline const Eigen::MatrixXd & jacobian() const noexcept { return mat_.matrix(); }

  inline const Eigen::MatrixXd & JDot() const noexcept { return mat_.matrixDot(); }

  inline const Robot & robot() const noexcept { return com_.robot(); }

  inline Robot & robot() noexcept { return com_.robot(); }

private:
  CoM & com_;
  rbd::CentroidalMomentumMatrix mat_;

  sva::ForceVecd momentum_;
  void updateMomentum();

  sva::ForceVecd velocity_ = sva::ForceVecd::Zero();

  sva::ForceVecd normalAcceleration_;
  void updateNormalAcceleration();

  void updateJacobian();

  void updateJDot();
};

} // namespace mc_tvm
