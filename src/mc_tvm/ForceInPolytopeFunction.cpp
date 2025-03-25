/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/ForceInPolytopeFunction.h>

#include <mc_tvm/Robot.h>

namespace mc_tvm
{

ForceInPolytopeFunction::ForceInPolytopeFunction(const mc_rbdyn::Contact & contact, const tvm::VariablePtr & forceVar, const mc_tvm::FeasiblePolytope & polytope)
: tvm::function::abstract::LinearFunction(polytope.offsets().size()), forceVar_(forceVar), polytope_(polytope)
{
  registerUpdates(Update::Jacobian, &ForceInPolytopeFunction::updateJacobian,
                  Update::B, &ForceInPolytopeFunction::updateb
                 );

  addOutputDependency<ForceInPolytopeFunction>(Output::Jacobian, Update::Jacobian);
  addOutputDependency<ForceInPolytopeFunction>(Output::B, Update::B);

  addVariable(forceVar, true);
  
  normals_ = polytope_.normals();
  offsets_ = polytope_.offsets();
  // Updates of the Jacobian and B depend on updating values of the feasible polytope
  // addInputDependency<ForceInPolytopeFunction>(Update::Jacobian, polytope_, FeasiblePolytope::Output::Polytope);
  // addInputDependency<ForceInPolytopeFunction>(Update::B, polytope_, FeasiblePolytope::Output::Polytope);

  // addInputDependency<ForceInPolytopeFunction>(Update::Velocity, forceVar_, mc_tvm::CoM::Output::Velocity);
  // addInputDependency<ForceInPolytopeFunction>(Update::Jacobian, forceVar_, mc_tvm::CoM::Output::Jacobian);
  // addInputDependency<ForceInPolytopeFunction>(Update::NormalAcceleration, forceVar_, mc_tvm::CoM::Output::NormalAcceleration);

  // addInternalDependency<ForceInPolytopeFunction>(Update::Velocity, Update::Jacobian);
  // addInternalDependency<ForceInPolytopeFunction>(Update::NormalAcceleration, Update::Velocity);
}

// void ForceInPolytopeFunction::addPlane(tvm::geometry::PlanePtr plane)
// {
//   planes_.push_back(plane);
//   addInputDependency<ForceInPolytopeFunction>(Update::Value, plane, tvm::geometry::Plane::Output::Position);
//   addInputDependency<ForceInPolytopeFunction>(Update::Velocity, plane, tvm::geometry::Plane::Output::Velocity);
//   addInputDependency<ForceInPolytopeFunction>(Update::NormalAcceleration, plane,
//                                           tvm::geometry::Plane::Output::Acceleration);
//   // resizing the dimension of the output value (here depends on the number of planes)
//   resize(static_cast<int>(planes_.size()));
// }

// void ForceInPolytopeFunction::reset()
// {
//   planes_.resize(0);
//   resize(0);
// }

// void ForceInPolytopeFunction::updateValue()
// {
//   value_ = polytope_.normals() * forceVar_->value() - polytope_.offsets();
// }

// void ForceInPolytopeFunction::updateVelocity()
// {
//   Eigen::DenseIndex i = 0;
//   for(const auto & p : planes_)
//   {
//     velocity_(i++) = p->normal().dot(com_.velocity() - p->speed()) + p->normalDot().dot(com_.com() - p->point());
//   }
// }

void ForceInPolytopeFunction::updateJacobian()
{
  jacobian_[forceVar_.get()] = normals_;
}

void ForceInPolytopeFunction::updateb()
{
  // Since the linear function expects format of Ax + b /operator/ rhs
  // the b member is minus the offsets
  b_ = - offsets_;
}

// void ForceInPolytopeFunction::updateNormalAcceleration()
// {
//   Eigen::DenseIndex i = 0;
//   for(const auto & p : planes_)
//   {
//     normalAcceleration_(i++) = p->normal().dot(com_.normalAcceleration() - p->acceleration())
//                                + 2 * p->normalDot().dot(com_.velocity() - p->speed())
//                                + p->normalDotDot().dot(com_.com() - p->point());
//   }
// }

} // namespace mc_tvm
