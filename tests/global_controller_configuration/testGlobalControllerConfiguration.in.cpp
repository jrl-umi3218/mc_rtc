/*
 * Copyright 2015-2024 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/mc_global_controller.h>

#include <mc_rbdyn/RobotLoader.h>

#include <boost/test/unit_test.hpp>

/** Make sure the robot loader has nothing available when we start a test case */
struct TestGlobalControllerConfigurationFixture
{
  TestGlobalControllerConfigurationFixture()
  {
    mc_rtc::Loader::debug_suffix = "";
    mc_rbdyn::RobotLoader::clear();
    std::cout << "AVAILABLE ROBOTS:\n";
    for(const auto & r : mc_rbdyn::RobotLoader::available_robots()) { std::cout << "- " << r << "\n"; }
    BOOST_REQUIRE(mc_rbdyn::RobotLoader::available_robots().empty());
  }
};

BOOST_FIXTURE_TEST_CASE(MainRobotSimple, TestGlobalControllerConfigurationFixture)
{
  mc_control::MCGlobalController::GlobalConfiguration cfg("@MAIN_ROBOT_SIMPLE_CFG@", nullptr, true);
  BOOST_REQUIRE(cfg.main_robot_module);
  BOOST_REQUIRE(cfg.main_robot_module->name == "jvrc1");
}

BOOST_FIXTURE_TEST_CASE(MainRobotSimpleVector, TestGlobalControllerConfigurationFixture)
{
  mc_control::MCGlobalController::GlobalConfiguration cfg("@MAIN_ROBOT_SIMPLE_VECTOR_CFG@", nullptr, true);
  BOOST_REQUIRE(cfg.main_robot_module);
  BOOST_REQUIRE(cfg.main_robot_module->name == "jvrc1");
}

BOOST_FIXTURE_TEST_CASE(MainRobotObject, TestGlobalControllerConfigurationFixture)
{
  mc_control::MCGlobalController::GlobalConfiguration cfg("@MAIN_ROBOT_OBJECT_CFG@", nullptr, true);
  BOOST_REQUIRE(cfg.main_robot_module);
  BOOST_REQUIRE(cfg.main_robot_module->name == "jvrc1");
}

BOOST_FIXTURE_TEST_CASE(MainRobotObjectVector, TestGlobalControllerConfigurationFixture)
{
  mc_control::MCGlobalController::GlobalConfiguration cfg("@MAIN_ROBOT_OBJECT_VECTOR_CFG@", nullptr, true);
  BOOST_REQUIRE(cfg.main_robot_module);
  BOOST_REQUIRE(cfg.main_robot_module->name == "jvrc1");
}
