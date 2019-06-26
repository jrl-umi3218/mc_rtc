/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "ConfigureState.h"

void ConfigureState::configure(const mc_rtc::Configuration & config)
{
  config("value", value_);
}

EXPORT_SINGLE_STATE("ConfigureState", ConfigureState)
