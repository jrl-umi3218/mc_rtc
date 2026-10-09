/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/ForceInPolytopeFunction.h>

#include <mc_tvm/FeasiblePolytope.h>

namespace mc_tvm
{

ForceInPolytopeFunction::ForceInPolytopeFunction(const mc_rbdyn::Contact & contact,
                                                 const tvm::VariableVector & forceVars,
                                                 const int & rIndex,
                                                 const bool & hasForceVar)
: tvm::function::abstract::LinearFunction(0), contact_(contact), rIndex_(rIndex), hasForceVar_(hasForceVar),
  tvmPoly_(rIndex == contact.r1Index() ? contact.tvmPolytopeR1() : contact.tvmPolytopeR2()), forceVars_(forceVars),
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
  addInputDependency<ForceInPolytopeFunction>(Update::Resize, tvmPoly_, FeasiblePolytope::Output::Polytope);

  // Resizing dimension of the function to number of polytope planes (must be done every new polytope)
  tvmPoly_.updatePolytope();
  resize(tvmPoly_.offsets().size());
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
      // The force only normals are the right 3 columns, minus bottom 12 rows (CoP + yaw torque)
      int nbOfForceConstraints = tvmPoly_.offsets().size() - 12;
      // FIXME Not handling other side variable in force case (very different logic,
      // all force vars should be associated to another etc)
      jacobian_[forceVar.get()] = tvmPoly_.normals().block(0, 3, nbOfForceConstraints, 3);
    }
    else if(forceVar->space().size() == 6)
    {
      if(hasForceVar_) { jacobian_[forceVar.get()] = tvmPoly_.normals(); }
      else
      {
        // if the force var was created for the other side we need to multiply the var by minus the
        // dual plücker transform matrix (manipulating a force vec) to get the wrench for this side

        // getting the right transform: if this robot is r1, the var is in frame r2 so we need X_r2_r1
        const auto dualMat =
            contact_.r1Index() == rIndex_ ? contact_.X_r2s_r1s().dualMatrix() : contact_.X_r2s_r1s().inv().dualMatrix();
        jacobian_[forceVar.get()] = tvmPoly_.normals() * -dualMat;
      }
    }
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
    // If the vars are force only, remove the last 12 elements of offsets (CoP + yaw torque)
    int nbOfForceConstraints = tvmPoly_.offsets().size() - 12;
    b_ = -tvmPoly_.offsets().segment(0, nbOfForceConstraints);
  }
  else if(forceVars_[0]->space().size() == 6) { b_ = -tvmPoly_.offsets(); }
  // mc_rtc::log::critical("b task updated correctly to {}", b_.transpose());
}

void ForceInPolytopeFunction::resizeToPoly()
{
  // If polytope number of planes changed, update the task dimension
  // The dimension is the number of constraints : it is both the jacobian rows and the output space
  if(forceVars_[0]->space().size() == 3)
  {
    // If we manipulate forces only, we don't need the 6D related constraint (CoP, torsional friction)
    if(imageSpace().size() != tvmPoly_.offsets().size() - 12)
    {
      // mc_rtc::log::warning("Image space changed from {} to {}", imageSpace().size(), tvmPoly_.offsets().size() - 12);
      resize(tvmPoly_.offsets().size() - 12);
      constraintSizeChanged_ = true;
    }
  }
  else if(forceVars_[0]->space().size() == 6)
  {
    if(imageSpace().size() != tvmPoly_.offsets().size())
    {
      // mc_rtc::log::warning("Image space changed from {} to {}", imageSpace().size(), tvmPoly_.offsets().size());
      resize(tvmPoly_.offsets().size());
      constraintSizeChanged_ = true;
    }
  }
}

} // namespace mc_tvm
