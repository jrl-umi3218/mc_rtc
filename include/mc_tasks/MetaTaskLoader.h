/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_solver/GenericLoader.h>
#include <mc_tasks/MetaTask.h>

namespace mc_tasks
{

struct MC_TASKS_DLLAPI MetaTaskLoader : public mc_solver::GenericLoader<MetaTaskLoader, MetaTask>
{
  static storage_t & storage();
};

} // namespace mc_tasks
