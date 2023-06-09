/*
 * Copyright 2023 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifdef BOOST_TEST_MAIN
#  undef BOOST_TEST_MAIN
#endif

#include <mc_observers/ObserverMacros.h>

#include <boost/test/unit_test.hpp>

struct TestObserver : public mc_observers::Observer
{

  using Observer::Observer;

  void configure(const mc_control::MCController &, const mc_rtc::Configuration & config) override
  {
    BOOST_REQUIRE(config.has("fromObserverConfig"));
    if(!config.has("robot")) { BOOST_REQUIRE(config.has("fromRobotConfig")); }
    if(name() == "FullConfiguration") { BOOST_REQUIRE(config.has("fromControllerConfig")); }
    BOOST_REQUIRE(config.has("finalValue"));
    if(name() == "FullConfiguration") { BOOST_REQUIRE(config("finalValue").operator int() == 3); }
    else if(name() == "NoControllerConfiguration") { BOOST_REQUIRE(config("finalValue").operator int() == 2); }
    else
    {
      BOOST_REQUIRE(name() == "NoControllerOrRobotConfiguration");
      BOOST_REQUIRE(config("finalValue").operator int() == 1);
    }
    desc_ = fmt::format("TestObserver ({})", name());
  }

  void reset(const mc_control::MCController &) override {}

  bool run(const mc_control::MCController &) override { return true; }

  void update(mc_control::MCController &) override {}
};

EXPORT_OBSERVER_MODULE("TestObserver", TestObserver)
