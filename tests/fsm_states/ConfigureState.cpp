/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "ConfigureState.h"

#include <boost/test/unit_test.hpp>

std::string ConfigureState::ExpectedStateName = "";

void ConfigureState::configure(const mc_rtc::Configuration & config)
{
  BOOST_CHECK_EQUAL(this->name(), ExpectedStateName);
  config("value", value_);
}

EXPORT_SINGLE_STATE("ConfigureState", ConfigureState)
