/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/DistanceLimit.h>
#include <mc_rtc/deprecated.h>

namespace mc_rbdyn
{

MC_RTC_DEPRECATED struct Collision : public DistanceLimit
{
  using DistanceLimit::DistanceLimit;
};

} // namespace mc_rbdyn
