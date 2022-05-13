/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tvm/api.h>

#include <mc_rbdyn/CompoundJointConstraintDescription.h>
#include <mc_rbdyn/fwd.h>

#include <tvm/function/abstract/LinearFunction.h>

namespace mc_tvm
{

/** This class implements a compound joint linear function
 *
 * For a given compound joint description it computes the distance to the
 * compound line
 */
struct MC_TVM_DLLAPI CompoundJointFunction : tvm::function::abstract::LinearFunction
{
  SET_UPDATES(CompoundJointFunction, B)

  /** Constructor
   *
   * Creates a compound joint function
   */
  CompoundJointFunction(const mc_rbdyn::Robot & robot, const mc_rbdyn::CompoundJointConstraintDescription & desc);

  /** Sets the timestep */
  void dt(double dt);

private:
  void updateB();

  // Robot
  mc_rbdyn::ConstRobotPtr robot_;
  // Timestep
  double dt_;
  // Simplified form of the description
  struct Desc
  {
    // Index of q1
    size_t q1Idx;
    // Index of q2
    size_t q2Idx;
    // x index of p1
    double p1_x;
    // y index of p1
    double p1_y;
    // P_x = p2_x - p1_x
    double P_x;
    // P_y = p2_y - p1_y
    double P_y;
  };
  Desc desc_;
  // Constant part of b
  double b_cst_;
};

using CompoundJointFunctionPtr = std::shared_ptr<CompoundJointFunction>;

} // namespace mc_tvm
