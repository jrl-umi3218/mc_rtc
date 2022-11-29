/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/ComplianceTask.h>

namespace mc_tasks
{

namespace force
{

static std::pair<double, double> defaultFGain = mc_tasks::force::ComplianceTask::defaultFGain;
static std::pair<double, double> defaultTGain = mc_tasks::force::ComplianceTask::defaultTGain;

} // namespace force

} // namespace mc_tasks
