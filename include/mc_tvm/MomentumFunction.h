/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tvm/Momentum.h>

#include <mc_rbdyn/fwd.h>

#include <tvm/function/abstract/Function.h>

#include <SpaceVecAlg/SpaceVecAlg>

namespace mc_tvm
{

/** This class implements a Momentum function for a given robot
 *
 * You can provide:
 * - a reference Momentum target
 * - a reference Momentum velocity target
 * - a reference Momentum acceleration target
 */
struct MC_TVM_DLLAPI MomentumFunction : tvm::function::abstract::Function
{
  SET_UPDATES(MomentumFunction, Value, Velocity, Jacobian, NormalAcceleration, JDot)

  /** Constructor
   *
   * Creates a Momentum function, the objective is the current robot's Momentum
   */
  MomentumFunction(const mc_rbdyn::Robot & robot);

  /** Reset the objectve to the current momentum and reference speed/acceleration to zero */
  void reset();

  /** Get the current objective */
  inline const sva::ForceVecd & momentum() const noexcept { return momentum_; }

  /** Set the objective */
  inline void momentum(const sva::ForceVecd & momentum) noexcept { momentum_ = momentum; }

  /** Get the current objective */
  inline const Eigen::Vector6d & refVel() const noexcept { return refVel_; }

  /** Set the objective */
  inline void refVel(const Eigen::Vector6d & refVel) noexcept { refVel_ = refVel; }

  /** Get the current objective */
  inline const Eigen::Vector6d & refAccel() const noexcept { return refAccel_; }

  /** Set the objective */
  inline void refAccel(const Eigen::Vector6d & refAccel) noexcept { refAccel_ = refAccel; }

  /** Get the associated algorithm */
  inline const mc_tvm::Momentum & algo() const noexcept { return momentumAlgo_; }

private:
  mc_tvm::Momentum & momentumAlgo_;
  sva::ForceVecd momentum_;
  Eigen::Vector6d refVel_;
  Eigen::Vector6d refAccel_;

  void updateValue();
  void updateVelocity();
  void updateJacobian();
  void updateNormalAcceleration();
  void updateJDot();
};

} // namespace mc_tvm
