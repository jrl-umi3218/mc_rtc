#include <mc_tasks/MetaTaskLoader.h>

namespace mc_tasks
{

MetaTaskLoader::storage_t & MetaTaskLoader::storage()
{
  static storage_t storage_;
  return storage_;
}

} // namespace mc_tasks
