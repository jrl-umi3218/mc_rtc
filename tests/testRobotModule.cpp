/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "utils.h"

#include <mc_rbdyn/Robots.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#include <boost/test/unit_test.hpp>

#ifndef JVRC_DESCRIPTION_PATH
#  error "JVRC_DESCRIPTION_PATH must be defined to build this RobotModule"
#endif

#define JVRC_VAL(x) #x
#define JVRC_VAL_VAL(x) JVRC_VAL(x)

static std::string JVRC1_DATA = fmt::format(R"(
path: "{}"
name: jvrc1
fixed: false
default_attitude: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8275]
forceSensors:
- name: RightFootForceSensor
  parentBody: R_ANKLE_P_S
  X_p_f:
    rotation: [0, 0, 0]
    translation: [0, 0, 0]
- name: LeftFootForceSensor
  parentBody: L_ANKLE_P_S
  X_p_f:
    rotation: [0, 0, 0]
    translation: [0, 0, 0]
- name: RightHandForceSensor
  parentBody: R_WRIST_Y_S
  X_p_f:
    rotation: [0, 0, 0]
    translation: [0, 0, 0]
- name: LeftHandForceSensor
  parentBody: L_WRIST_Y_S
  X_p_f:
    rotation: [0, 0, 0]
    translation: [0, 0, 0]
bodySensors:
- name: Accelerometer
  parentBody: PELVIS_S
  X_b_s:
    rotation: [0, 0, 0]
    translation: [0, 0, 0]
- name: FloatingBase
  parentBody: PELVIS_S
  X_b_s:
    rotation: [0, 0, 0]
    translation: [0, 0, 0]
grippers:
- name: l_gripper
  joints: [L_UTHUMB]
  reverse_limits: true
- name: r_gripper
  joints: [R_UTHUMB]
  reverse_limits: false
frames:
- name: Camera
  parent: NECK_P_S
  X_p_f:
    translation: [0, 0, 0.1]
    rotation: [3.14, 0, 3.14]
- name: Camera_RGB
  parent: Camera_Depth
  X_p_f:
    translation: [-0.2, 0, 0]
- name: Camera_Depth
  parent: Camera
  X_p_f:
    translation: [0.1, 0, 0]
)",
                                            JVRC_VAL_VAL(JVRC_DESCRIPTION_PATH));

BOOST_AUTO_TEST_CASE(TestRobotModule)
{
  auto config = makeConfigFile(JVRC1_DATA, ".yaml");
  configureRobotLoader();
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("json", config);
  bfs::remove(config);
  auto robots = mc_rbdyn::loadRobot(*rm);
  const auto & robot = robots->robot();

  // Check frames loading
  for(const auto & b : robot.mb().bodies())
  {
    BOOST_REQUIRE(robot.hasFrame(b.name()));
  }
  for(const auto & s : robot.surfaces())
  {
    BOOST_REQUIRE(robot.hasFrame(s.first));
  }
  for(const auto & f : {"Camera", "Camera_RGB", "Camera_Depth"})
  {
    BOOST_REQUIRE(robot.hasFrame(f));
  }
  BOOST_REQUIRE(robot.frame("Camera").body() == "NECK_P_S");
  BOOST_REQUIRE(robot.frame("Camera").parent());
  BOOST_REQUIRE(robot.frame("Camera").parent()->name() == "NECK_P_S");
  BOOST_REQUIRE(robot.frame("Camera_Depth").body() == "NECK_P_S");
  BOOST_REQUIRE(robot.frame("Camera_Depth").parent());
  BOOST_REQUIRE(robot.frame("Camera_Depth").parent()->name() == "Camera");
  BOOST_REQUIRE(robot.frame("Camera_RGB").body() == "NECK_P_S");
  BOOST_REQUIRE(robot.frame("Camera_RGB").parent());
  BOOST_REQUIRE(robot.frame("Camera_RGB").parent()->name() == "Camera_Depth");

  // Check grippers
  BOOST_REQUIRE(robot.grippers().size() == 2);
  BOOST_REQUIRE(robot.hasGripper("l_gripper"));
  BOOST_REQUIRE(robot.hasGripper("r_gripper"));

  // Check force sensors
  BOOST_REQUIRE(robot.forceSensors().size() == 4);
  for(const auto & fs : {"RightFootForceSensor", "LeftFootForceSensor", "RightHandForceSensor", "LeftHandForceSensor"})
  {
    BOOST_REQUIRE(robot.hasForceSensor(fs));
  }

  // Check body sensors
  BOOST_REQUIRE(robot.bodySensors().size() == 2);
  for(const auto & bs : {"Accelerometer", "FloatingBase"})
  {
    BOOST_REQUIRE(robot.hasBodySensor(bs));
  }
}
