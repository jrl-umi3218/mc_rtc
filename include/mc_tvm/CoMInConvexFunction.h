/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tvm/CoM.h>
#include <mc_tvm/Robot.h>

#include <mc_rbdyn/fwd.h>

#include <tvm/function/abstract/Function.h>
#include <tvm/geometry/Plane.h>

namespace mc_tvm
{

/** This function computes the distance of the CoM to a set of planes.
 *
 * By providing a consistent set of planes, the function can be used to keep
 * the CoM in a convex region of space.
 */
struct MC_TVM_DLLAPI CoMInConvexFunction : public tvm::function::abstract::Function
{
public:
  using Output = tvm::function::abstract::Function::Output;
  DISABLE_OUTPUTS(Output::JDot)
  SET_UPDATES(CoMInConvexFunction, Value, Velocity, Jacobian, NormalAcceleration)

  /** Constructor
   *
   * By default, this function computes nothing
   *
   */
  CoMInConvexFunction(const mc_rbdyn::Robot & robot);

  /** Add a plane.
   *
   * This will add one dimension to the function output. This new value is
   * the distance to that plane.
   *
   */
  void addPlane(tvm::geometry::PlanePtr plane);

  /** Remove all planes */
  void reset();

  /** Access the robot this function uses */
  inline const mc_rbdyn::Robot & robot() const noexcept { return com_.robot().robot(); }

  /** Access the planes used by this function */
  inline const std::vector<tvm::geometry::PlanePtr> & planes() noexcept { return planes_; }

  /** Access a pane at the given index */
  inline tvm::geometry::Plane & plane(size_t i) noexcept
  {
    assert(i < planes_.size() && planes_[i]);
    return *planes_[i];
  }

protected:
  void updateValue();
  void updateVelocity();
  void updateJacobian();
  void updateNormalAcceleration();

  mc_tvm::CoM & com_;

  /** Set of planes */
  std::vector<tvm::geometry::PlanePtr> planes_;
};

using CoMInConvexFunctionPtr = std::shared_ptr<CoMInConvexFunction>;

} // namespace mc_tvm
