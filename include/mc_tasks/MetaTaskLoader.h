#pragma once

#include <mc_tasks/MetaTask.h>

#include <mc_solver/GenericLoader.h>

namespace mc_tasks
{
  using MetaTaskLoader = mc_solver::GenericLoader<MetaTask>;
}
