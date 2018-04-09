#include "ConfigureState.h"

void ConfigureState::configure(const mc_rtc::Configuration & config)
{
  config("value", value_);
}

EXPORT_SINGLE_STATE("ConfigureState", ConfigureState)
