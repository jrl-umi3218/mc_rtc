/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tvm/Robot.h>
#include <mc_tvm/FeasiblePolytope.h>

#include <mc_rbdyn/fwd.h>

#include <tvm/function/abstract/LinearFunction.h>
#include <tvm/geometry/Plane.h>

namespace mc_tvm
{

/** This is a linear function to constraint a variable in a set of planes
 *
 * By providing a set of planes, the function can be used to keep the
 * variable in the convex polytope (then the function rhs must be set to <= 0.0)
 * 
 * It is written to constraint a force in its feasible polytope but can be applied to any case
 */
struct MC_TVM_DLLAPI ForceInPolytopeFunction : public tvm::function::abstract::LinearFunction
{
public:
  using Output = tvm::function::abstract::LinearFunction::Output;
  DISABLE_OUTPUTS(Output::JDot)
  // Since this is a linear function no need to update the value compared to the regular functions
  // We expect the format Ax <= b, a linear function enforces Jac * var(s) + B /operator/ rhs
  // This means we need to update Jacobian and B depending on the normals and offsets of the poly
  SET_UPDATES(ForceInPolytopeFunction, Jacobian, B)

  /** Constructor
   */
  ForceInPolytopeFunction(const mc_rbdyn::Contact & contact, const tvm::VariablePtr & forceVar, const mc_tvm::FeasiblePolytope & polytope);

  /** Resize constraint to polytope size */
  void reset();

  /** Access the polytope used by this function */
  inline const mc_tvm::FeasiblePolytope & polytope() noexcept { return polytope_; }

protected:
  void updateJacobian();
  void updateb();

  tvm::VariablePtr forceVar_;

  /** Feasible polytope */
  const mc_tvm::FeasiblePolytope & polytope_;

  Eigen::MatrixX3d normals_;
  Eigen::VectorXd offsets_;

};

using ForceInPolytopeFunctionPtr = std::shared_ptr<ForceInPolytopeFunction>;

} // namespace mc_tvm
