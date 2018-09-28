#pragma once

#include <mc_solver/ConstraintSet.h>
#include <mc_solver/GenericLoader.h>

namespace mc_solver
{

struct MC_SOLVER_DLLAPI ConstraintSetLoader : public mc_solver::GenericLoader<ConstraintSetLoader, ConstraintSet>
{
  static storage_t & storage();
};

} // namespace mc_solver
