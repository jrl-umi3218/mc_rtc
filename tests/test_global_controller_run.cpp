/*
 * Copyright 2015-2023 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/Ticker.h>

#include <boost/test/unit_test.hpp>

#include "test_global_controller_config.h"
#include "utils.h"

static bool initialized = configureRobotLoader();

BOOST_AUTO_TEST_CASE(RUN)
{
  mc_control::Ticker::Configuration config;
  config.mc_rtc_configuration = get_config_file();
  mc_control::Ticker ticker(config);
  auto do_sim_loops = [&]()
  {
    for(size_t i = 0; i < nrIter(); ++i)
    {
      bool r = ticker.step();
      if(!r) { mc_rtc::log::critical("Failed at iter {}", i); }
      BOOST_REQUIRE(r);
    }
  };
  do_sim_loops();
  if(next_controller() != "")
  {
    BOOST_REQUIRE(ticker.controller().EnableController(next_controller()));
    do_sim_loops();
  }
}
