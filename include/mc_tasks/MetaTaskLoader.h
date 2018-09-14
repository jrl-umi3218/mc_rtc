#pragma once

#include <mc_tasks/MetaTask.h>

#include <mc_solver/GenericLoader.h>

namespace mc_tasks
{

  struct MC_TASKS_DLLAPI MetaTaskLoader : public mc_solver::GenericLoader<MetaTaskLoader, MetaTask>
  {
    static storage_t & storage();
  };

}
