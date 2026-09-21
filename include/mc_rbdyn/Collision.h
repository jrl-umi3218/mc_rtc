/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/DistanceLimit.h>

namespace mc_rbdyn
{

using Collision [[deprecated("Use DistanceLimit instead.")]] = DistanceLimit;

} // namespace mc_rbdyn
