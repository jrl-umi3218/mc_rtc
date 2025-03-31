/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/ForceInPolytopeFunction.h>

#include <mc_tvm/FeasiblePolytope.h>

namespace mc_tvm
{

ForceInPolytopeFunction::ForceInPolytopeFunction(const mc_rbdyn::Contact & contact,
                                                 const tvm::VariableVector & forceVars,
                                                 const double & dir)
: tvm::function::abstract::LinearFunction(0), contact_(contact), forceVars_(forceVars), dir_(dir),
  constraintSizeChanged_(false)
{
  registerUpdates(Update::Jacobian, &ForceInPolytopeFunction::updateJacobian);
  registerUpdates(Update::B, &ForceInPolytopeFunction::updateb);
  registerUpdates(Update::Resize, &ForceInPolytopeFunction::resizeToPoly);

  addOutputDependency<ForceInPolytopeFunction>(Output::Jacobian, Update::Jacobian);
  addOutputDependency<ForceInPolytopeFunction>(Output::B, Update::B);

  // Adding all variables in the vector
  addVariable(forceVars, true);

  // Make sure the function has the right dimension
  addInternalDependency<ForceInPolytopeFunction>(Update::Jacobian, Update::Resize);
  addInternalDependency<ForceInPolytopeFunction>(Update::B, Update::Resize);

  // Updating Jacobian and B (and resizing) depends on updating the polytope
  auto & tvmPoly = contact.tvmPolytope();
  addInputDependency<ForceInPolytopeFunction>(Update::Resize, tvmPoly, FeasiblePolytope::Output::Polytope);

  // Resizing dimension of the function to number of polytope planes (must be done every new polytope)
  tvmPoly.updatePolytope();
  resize(tvmPoly.offsets().size());
  updateJacobian();
  updateb();
}

void ForceInPolytopeFunction::updateJacobian()
{
  // The function must bound the total of forces acting on this contact in the contact polytope
  // If the force vars dimensions are 3d, then we bound in the force part of the polytope
  // If the var dimension is 6d, we use the full wrench polytope
  for(const auto & forceVar : forceVars_)
  {
    if(forceVar->space().size() == 3)
    {
      // The force only normals are the right 3 columns, minus bottom 4 rows (CoP)
      int nbOfForceConstraints = contact_.tvmPolytope().offsets().size() - 4;
      jacobian_[forceVar.get()] = dir_ * contact_.tvmPolytope().normals().block(0, 3, nbOfForceConstraints, 3);
    }
    else if(forceVar->space().size() == 6) { jacobian_[forceVar.get()] = dir_ * contact_.tvmPolytope().normals(); }
    // Not handling other dimensions

    // mc_rtc::log::critical("Jacobian task {} updated correctly to\n{}", forceVars_.indexOf(*forceVar.get()),
    //                       jacobian_[forceVar.get()]);
  }
}

void ForceInPolytopeFunction::updateb()
{
  // Since the linear function expects format of Ax + b /operator/ rhs
  // the b member is minus the offsets

  if(forceVars_[0]->space().size() == 3) // check done on 1st var only, we assume all vars have same dim
  {
    // If the vars are force only, remove the last 4 elements of offsets (CoP)
    int nbOfForceConstraints = contact_.tvmPolytope().offsets().size() - 4;
    b_ = dir_ * -contact_.tvmPolytope().offsets().segment(0, nbOfForceConstraints);
  }
  else if(forceVars_[0]->space().size() == 6) { b_ = dir_ * -contact_.tvmPolytope().offsets(); }
  // mc_rtc::log::critical("b task updated correctly to {}", b_.transpose());
}

void ForceInPolytopeFunction::resizeToPoly()
{
  // If polytope number of planes changed, update the task dimension
  if(forceVars_[0]->space().size() == 3)
  {
    if(imageSpace().size() != contact_.tvmPolytope().offsets().size() - 4)
    {
      // mc_rtc::log::warning("Image space changed from {} to {}", imageSpace().size(),
      //                     contact_.tvmPolytope().offsets().size() - 4);
      resize(contact_.tvmPolytope().offsets().size() - 4);
      constraintSizeChanged_ = true;
    }
  }
  else if(forceVars_[0]->space().size() == 6)
  {
    if(imageSpace().size() != contact_.tvmPolytope().offsets().size())
    {
      // mc_rtc::log::warning("Image space changed from {} to {}", imageSpace().size(),
      //                       contact_.tvmPolytope().offsets().size());
      resize(contact_.tvmPolytope().offsets().size());
      constraintSizeChanged_ = true;
    }
  }
}

} // namespace mc_tvm
