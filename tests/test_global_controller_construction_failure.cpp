/*
 * Copyright 2015-2023 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/Ticker.h>

#include <boost/test/unit_test.hpp>

#include "test_global_controller_config.h"
#include "utils.h"

static bool initialized = configureRobotLoader();

BOOST_AUTO_TEST_CASE(CONSTRUCTION_FAILURE)
{
  BOOST_REQUIRE_THROW(mc_control::MCGlobalController{get_config_file()}, std::exception);
}
