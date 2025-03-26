/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/ForceInPolytopeFunction.h>

#include <mc_tvm/FeasiblePolytope.h>

namespace mc_tvm
{

ForceInPolytopeFunction::ForceInPolytopeFunction(const mc_rbdyn::Contact & contact,
                                                 const tvm::VariablePtr & forceVar,
                                                 const double & dir)
: tvm::function::abstract::LinearFunction(0), contact_(contact), forceVar_(forceVar), dir_(dir)
{
  registerUpdates(Update::Jacobian, &ForceInPolytopeFunction::updateJacobian);
  registerUpdates(Update::B, &ForceInPolytopeFunction::updateb);
  registerUpdates(Update::Resize, &ForceInPolytopeFunction::resizeToPoly);

  addOutputDependency<ForceInPolytopeFunction>(Output::Jacobian, Update::Jacobian);
  addOutputDependency<ForceInPolytopeFunction>(Output::B, Update::B);

  addVariable(forceVar, true);

  // Make sure the function has the right dimension
  addInternalDependency<ForceInPolytopeFunction>(Update::Jacobian, Update::Resize);
  addInternalDependency<ForceInPolytopeFunction>(Update::B, Update::Resize);

  // Updating Jacobian and B (and resizing) depends on updating the polytope
  auto & tvmPoly = contact.tvmPolytope();
  addInputDependency<ForceInPolytopeFunction>(Update::Resize, tvmPoly, FeasiblePolytope::Output::Polytope);

  // addInputDependency<ForceInPolytopeFunction>(Update::B, polytope_, FeasiblePolytope::Output::Polytope);

  // addInputDependency<ForceInPolytopeFunction>(Update::Velocity, forceVar_, mc_tvm::CoM::Output::Velocity);
  // addInputDependency<ForceInPolytopeFunction>(Update::Jacobian, forceVar_, mc_tvm::CoM::Output::Jacobian);
  // addInputDependency<ForceInPolytopeFunction>(Update::NormalAcceleration, forceVar_,
  // mc_tvm::CoM::Output::NormalAcceleration);

  // Resizing dimension of the function to number of polytope planes (must be done every new polytope)
  resize(tvmPoly.offsets().size());
  // updateJacobian();
  // updateb();
}

void ForceInPolytopeFunction::updateJacobian()
{
  jacobian_[forceVar_.get()] = dir_ * contact_.tvmPolytope().normals();
}

void ForceInPolytopeFunction::updateb()
{
  // Since the linear function expects format of Ax + b /operator/ rhs
  // the b member is minus the offsets
  b_ = dir_ * -contact_.tvmPolytope().offsets();
}

void ForceInPolytopeFunction::resizeToPoly()
{
  // Polytope number of planes changed, update the task dimension
  if(imageSpace() != contact_.tvmPolytope().offsets().size()) { resize(contact_.tvmPolytope().offsets().size()); }
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
