#include <mc_control/SimulationContactSensor.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rtc/config.h>

#include <boost/test/unit_test.hpp>

mc_rbdyn::Robots & get_robots()
{
  static std::shared_ptr<mc_rbdyn::Robots> robots_ptr = nullptr;
  if(robots_ptr)
  {
    return *robots_ptr;
  }
  mc_rbdyn::RobotLoader::clear();
  mc_rbdyn::RobotLoader::update_robot_module_path({"@CMAKE_CURRENT_BINARY_DIR@/../src/mc_robots"});
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC-1");
  auto env = mc_rbdyn::RobotLoader::get_robot_module("env", std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH),
                                                     std::string("ground"));
  robots_ptr = mc_rbdyn::loadRobots({rm, env});
  return *robots_ptr;
}

BOOST_AUTO_TEST_CASE(TestSimulationContactPair)
{
  auto & robots = get_robots();
  auto & robot = robots.robot();
  auto & env = robots.env();

  mc_control::SimulationContactPair pair(robot.surfaces().at("LeftFoot"), env.surfaces().at("AllGround"));

  // In its default configuration the robot's foot should be at the ground
  // level
  BOOST_REQUIRE_SMALL(pair.update(robot, env), 1e-6);
  // 1 m above the ground
  robot.mbc().q[0].back() += 1.0;
  robot.forwardKinematics();
  BOOST_REQUIRE(pair.update(robot, env) > 0.99);
  // ~1 mm below the ground
  robot.mbc().q[0].back() -= 1.0 + 1e-3;
  robot.forwardKinematics();
  BOOST_REQUIRE(pair.update(robot, env) < 0);
}
