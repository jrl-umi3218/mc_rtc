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

/** This class implements a function to compute the local velocity of a given frame */
struct MC_TVM_DLLAPI FrameVelocity : public tvm::function::abstract::Function
{
  SET_UPDATES(FrameVelocity, Value, Velocity, Jacobian)

  /** Constructor
   *
   * Creates a FrameVelocity function, values are correctly initialized
   */
  FrameVelocity(const mc_rbdyn::RobotFrame & frame, const Eigen::Vector6d & dof);

  /** Empty reset function in case we want to wrap this as a task later */
  inline void reset() {}

  /** Access the related frame */
  inline const mc_rbdyn::RobotFrame & frame() const noexcept
  {
    return *frame_;
  }

  /** Get the dof vector */
  inline const Eigen::Vector6d & dof() const noexcept
  {
    return dof_;
  }

  /** Set the dof vector */
  inline void dof(const Eigen::Vector6d & dof) noexcept
  {
    dof_ = dof;
  }

  /** Set the reference velocity */
  inline void refVel(const Eigen::Vector6d & refV) noexcept
  {
    refVel_ = refV;
  }
  /** Get the reference velocity */
  inline const Eigen::Vector6d & refVel() const noexcept
  {
    return refVel_;
  }

  /** Set the reference acceleration */
  inline void refAccel(const Eigen::Vector6d & refA) noexcept
  {
    refAccel_ = refA;
  }
  /** Get the reference acceleration */
  inline const Eigen::Vector6d & refAccel() const noexcept
  {
    return refAccel_;
  }

private:
  mc_rbdyn::ConstRobotFramePtr frame_;
  Eigen::Vector6d dof_;
  Eigen::Vector6d refVel_ = Eigen::Vector6d::Zero();
  Eigen::Vector6d refAccel_ = Eigen::Vector6d::Zero();

  rbd::Jacobian jac_;
  rbd::Blocks blocks_;
  Eigen::MatrixXd jacobian_;

  void updateValue();
  void updateVelocity();
  void updateJacobian();
};

using FrameVelocityPtr = std::shared_ptr<FrameVelocity>;

} // namespace mc_tvm
