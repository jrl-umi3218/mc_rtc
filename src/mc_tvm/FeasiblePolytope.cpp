/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/FeasiblePolytope.h>

namespace mc_tvm
{

FeasiblePolytope::FeasiblePolytope(NewPolytopeToken, const mc_rbdyn::Contact & contact, const int & rIndex)
: contact_(contact), rIndex_(rIndex)
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
  // Use given robot index to know if this poly corresponds to r1 or r2 of the contact
  bool isR1 = contact_.r1Index() == rIndex_;
  std::optional<mc_rbdyn::FeasiblePolytope> feasiblePolytope;
  if(isR1) { feasiblePolytope = contact_.feasiblePolytopeR1(); }
  else { feasiblePolytope = contact_.feasiblePolytopeR2(); }

  if(feasiblePolytope)
  {
    // If there is a feasible polytope, build the contact wrench polytope from the wrench face matrix and
    // the feasible polytope
    // The moments are the 3 first columns, the forces the 3 next
    // Nb of lines: force poly constraints + 12 surface constraints (4 CoP, 8 yaw torque)
    // Note: the 12 surface constraints assume a rectangular surface !
    int nbPolyConstraints = feasiblePolytope->planeConstants.size();
    int nbSurfaceConstraints = 12;

    normals_ = Eigen::MatrixXd::Zero(nbPolyConstraints + nbSurfaceConstraints, 6);
    offsets_ = Eigen::VectorXd::Zero(nbPolyConstraints + nbSurfaceConstraints);

    // Fill polytope in force part only if it's a force polytope, in full matrix if it's a wrench polytope
    if(feasiblePolytope->planeNormals.cols() == 3)
    {
      normals_.block(0, 3, nbPolyConstraints, 3) = feasiblePolytope->planeNormals;
    }
    else if(feasiblePolytope->planeNormals.cols() == 6)
    {
      normals_.block(0, 0, nbPolyConstraints, 6) = feasiblePolytope->planeNormals;
    }
    else
    {
      mc_rtc::log::error("[TVM Feasible Polytope]: Invalid polytope for contact frame {}",
                         isR1 ? contact_.r1Surface()->name() : contact_.r2Surface()->name());
    }

    // For surface constraints use the correct robot's surface
    if(isR1)
    {
      normals_.block(nbPolyConstraints, 0, nbSurfaceConstraints, 6) =
          computeSurfaceTorqueConstraint(*contact_.r1Surface(), contact_.friction());
    }
    else
    {
      normals_.block(nbPolyConstraints, 0, nbSurfaceConstraints, 6) =
          computeSurfaceTorqueConstraint(*contact_.r2Surface(), contact_.friction());
    }

    offsets_.segment(0, nbPolyConstraints) = feasiblePolytope->planeConstants;
    // Offsets for wrench face matrix are just zero, nothing else to do
    // mc_rtc::log::warning("Feasible Poly: feasible detected, built as:\n{}\n and {}", normals_, offsets_.transpose());
  }
  else
  {
    // There is no feasible polytope, build a default one with just friction cone and surface matrix
    // Note: the 12 surface constraints assume a rectangular surface !
    int nbFrictionSides = 5;
    int nbSurfaceConstraints = 12;

    normals_ = Eigen::MatrixXd::Zero(nbFrictionSides + nbSurfaceConstraints, 6);
    offsets_ = Eigen::VectorXd::Zero(nbFrictionSides + nbSurfaceConstraints);
    // Translational friction cones
    normals_.block(0, 3, nbFrictionSides, 3) =
        generatePolyhedralConeHRep(nbFrictionSides, Eigen::Matrix3d::Identity(), contact_.friction());
    // Surface constraints
    if(isR1) // Use correct robot's surface
    {
      normals_.block(nbFrictionSides, 0, nbSurfaceConstraints, 6) =
          computeSurfaceTorqueConstraint(*contact_.r1Surface(), contact_.friction());
    }
    else
    {
      normals_.block(nbFrictionSides, 0, nbSurfaceConstraints, 6) =
          computeSurfaceTorqueConstraint(*contact_.r2Surface(), contact_.friction());
    }

    // Offsets are all zero in this default case (no second member if unbound polyhedral cone)
    // mc_rtc::log::warning("Feasible Poly: no feasible detected, built default as:\n{}", normals_);
  }
}

} // namespace mc_tvm
