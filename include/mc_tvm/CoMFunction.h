/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tvm/CoM.h>

#include <mc_rbdyn/fwd.h>

#include <tvm/function/abstract/Function.h>

namespace mc_tvm
{

/** This class implements a CoM function for a given robot
 *
 * You can provide:
 * - a reference CoM target
 * - a reference CoM velocity target
 * - a reference CoM acceleration target
 */
struct MC_TVM_DLLAPI CoMFunction : tvm::function::abstract::Function
{
  SET_UPDATES(CoMFunction, Value, Velocity, Jacobian, NormalAcceleration, JDot)

  /** Constructor
   *
   * Creates a CoM function, the objective is the current robot's CoM
   */
  CoMFunction(const mc_rbdyn::Robot & robot);

  /** Reset the objective to the current CoM and reference speed/acceleration to zero */
  void reset();

  /** Get the current objective */
  inline const Eigen::Vector3d & com() const noexcept { return com_; }

  /** Set the objective */
  inline void com(const Eigen::Vector3d & com) noexcept { com_ = com; }

  /** Get the robot's current CoM */
  inline const Eigen::Vector3d & actual() const noexcept { return comAlgo_.com(); }

  /** Get the current objective */
  inline const Eigen::Vector3d & refVel() const noexcept { return refVel_; }

  /** Set the objective */
  inline void refVel(const Eigen::Vector3d & refVel) noexcept { refVel_ = refVel; }

  /** Get the current objective */
  inline const Eigen::Vector3d & refAccel() const noexcept { return refAccel_; }

  /** Set the objective */
  inline void refAccel(const Eigen::Vector3d & refAccel) noexcept { refAccel_ = refAccel; }

private:
  const mc_tvm::CoM & comAlgo_;
  Eigen::Vector3d com_;
  Eigen::Vector3d refVel_;
  Eigen::Vector3d refAccel_;

  void updateValue();
  void updateVelocity();
  void updateJacobian();
  void updateNormalAcceleration();
  void updateJDot();
};

} // namespace mc_tvm
