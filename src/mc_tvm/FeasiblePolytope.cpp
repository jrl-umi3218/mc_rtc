/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/FeasiblePolytope.h>

namespace mc_tvm
{

FeasiblePolytope::FeasiblePolytope(/*NewPolytopeToken, */const mc_rbdyn::Contact & contact) : contact_(contact)
{
  registerUpdates(Update::Polytope, &FeasiblePolytope::updatePolytope);
  
  // This makes it so that to update the polytope value, it will fetch the rbdyn contact polytope

  // addInputDependency(Update::Value, tvm_frame, Frame::Output::Position);
  // addOutputDependency(Output::Polytope, Update::Polytope);

  // Updating values from the start
  updatePolytope();

}

void FeasiblePolytope::updatePolytope()
{
  const auto & feasiblePolytope = contact_.feasiblePolytope();
  if(feasiblePolytope)
  {
    normals_ = feasiblePolytope->planeNormals;
    offsets_ = feasiblePolytope->planeConstants;
  }
}


} // namespace mc_tvm
