/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/FeasiblePolytope.h>

namespace mc_tvm
{

FeasiblePolytope::FeasiblePolytope(NewPolytopeToken, const mc_rbdyn::Contact & contact) : contact_(contact)
{
  registerUpdates(Update::Polytope, &FeasiblePolytope::updatePolytope);

  // This makes it so that to update the polytope value, it will fetch the rbdyn contact polytope
  addOutputDependency(Output::Polytope, Update::Polytope);
  // addInputDependency(Update::Value, tvm_frame, Frame::Output::Position);

  // Updating values from the start
  updatePolytope();
}

void FeasiblePolytope::updatePolytope()
{
  const auto feasiblePolytope = contact_.feasiblePolytope();
  if(feasiblePolytope)
  {
    // If there is a feasible polytope, build the contact wrench polytope from the wrench face matrix (moments part) and
    // the feasible polytope (forces part)
    // The moments are the 3 first columns, the forces the 3 next
    // Nb of lines: force poly constraints + 4 CoP constraints
    int nbForceConstraints = feasiblePolytope->planeConstants.size();
    int nbMomentConstraints = 4; // XXX 4 CoP constraints, see if need to include rest of wrench face mat
    normals_ = Eigen::MatrixXd::Zero(nbForceConstraints + nbMomentConstraints, 6);
    offsets_ = Eigen::VectorXd::Zero(nbForceConstraints + nbMomentConstraints);

    normals_.block(0, 3, nbForceConstraints, 3) = feasiblePolytope->planeNormals;
    // XXX see how to handle the contact not being from the first surface
    normals_.block(nbForceConstraints, 0, nbMomentConstraints, 6) = computeCoPMomentsConstraint(*contact_.r1Surface());

    offsets_.segment(0, nbForceConstraints) = feasiblePolytope->planeConstants;
    // Offsets for wrench face matrix are just zero, nothing else to do
    // mc_rtc::log::warning("Feasible Poly: feasible detected, built as:\n{}\n and {}", normals_, offsets_.transpose());
  }
  else
  {
    // There is no feasible polytope, build a default one with just friction cone and CoP matrix
    int nbFrictionSides = 5;
    int nbMomentConstraints = 4;

    normals_ = Eigen::MatrixXd::Zero(nbFrictionSides + nbMomentConstraints, 6);
    offsets_ = Eigen::VectorXd::Zero(nbFrictionSides + nbMomentConstraints);
    // Forces
    normals_.block(0, 3, nbFrictionSides, 3) =
        generatePolyhedralConeHRep(nbFrictionSides, Eigen::Matrix3d::Identity(), contact_.friction());
    // CoP moments
    // XXX see how to handle the contact not being from the first surface
    normals_.block(nbFrictionSides, 0, nbMomentConstraints, 6) = computeCoPMomentsConstraint(*contact_.r1Surface());

    // Offsets are all zero in this default case (no second member if unbound polyhedral cone)
    // mc_rtc::log::warning("Feasible Poly: no feasible detected, built default as:\n{}", normals_);
  }
}

} // namespace mc_tvm
