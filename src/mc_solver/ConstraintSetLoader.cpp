#include <mc_solver/ConstraintSetLoader.h>

namespace mc_solver
{

ConstraintSetLoader::storage_t & ConstraintSetLoader::storage()
{
  static storage_t storage_;
  return storage_;
}

} // namespace mc_solver
