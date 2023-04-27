/*
 * Copyright 2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "utils.h"

#include <mc_rbdyn/RobotConverter.h>
#include <mc_rbdyn/Robots.h>
#include <boost/test/unit_test.hpp>
#include <random>

static bool configured = configureRobotLoader();

BOOST_AUTO_TEST_CASE(TestCanonicalRobot)
{
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1NoHands");
  BOOST_REQUIRE(rm->_canonicalParameters.size());
  BOOST_REQUIRE(rm->_canonicalParameters[0] == "JVRC1");
  auto rmc = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");

  auto canonicalRobots = mc_rbdyn::loadRobot(*rmc);
  auto robots = mc_rbdyn::loadRobot(*rm);
  auto & canonicalRobot = canonicalRobots->robot();
  auto & robot = robots->robot();

  auto filteredLinks =
      std::vector<std::string>{"R_UTHUMB_S", "R_LTHUMB_S", "R_UINDEX_S", "R_LINDEX_S", "R_ULITTLE_S", "R_LLITTLE_S",
                               "L_UTHUMB_S", "L_LTHUMB_S", "L_UINDEX_S", "L_LINDEX_S", "L_ULITTLE_S", "L_LLITTLE_S"};

  for(const auto & filteredLink : filteredLinks)
  {
    BOOST_REQUIRE(canonicalRobot.hasBody(filteredLink));
    BOOST_REQUIRE(!robot.hasBody(filteredLink));
  }
  BOOST_REQUIRE(!robot.hasJoint("L_UTHUMB"));
  BOOST_REQUIRE(!robot.hasJoint("R_UTHUMB"));
  BOOST_REQUIRE(canonicalRobot.hasJoint("L_UTHUMB"));
  BOOST_REQUIRE(canonicalRobot.hasJoint("R_UTHUMB"));

  BOOST_REQUIRE(robot.mbc().q.size() != canonicalRobot.mbc().q.size());
  BOOST_REQUIRE(robot.mbc().alpha.size() != canonicalRobot.mbc().alpha.size());
  BOOST_REQUIRE(robot.mbc().alphaD.size() != canonicalRobot.mbc().alphaD.size());
  BOOST_REQUIRE(robot.mbc().jointTorque.size() != canonicalRobot.mbc().jointTorque.size());
}

BOOST_AUTO_TEST_CASE(TestCanonicalRobotConverter)
{
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1NoHands");
  BOOST_REQUIRE(rm->_canonicalParameters.size());
  BOOST_REQUIRE(rm->_canonicalParameters[0] == "JVRC1");
  auto rmc = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");

  auto robots = mc_rbdyn::loadRobot(*rm);
  auto & robot = robots->robot();

  auto canonicalRobots = mc_rbdyn::Robots::make();
  canonicalRobots->load(*rmc, mc_rbdyn::LoadRobotParameters{}.data(robot.data()));
  auto & canonicalRobot = canonicalRobots->robot();

  std::random_device dev;
  std::mt19937 rng(dev());
  std::uniform_real_distribution<> dist(0.1, 1); // distribution in range [0, 1]

  for(unsigned i = 0; i < robot.mbc().q.size(); ++i)
  {
    if(robot.mbc().q[i].size() == 1)
    {
      robot.mbc().q[i][0] = dist(rng);
      robot.mbc().alpha[i][0] = dist(rng);
      robot.mbc().alphaD[i][0] = dist(rng);
      robot.mbc().jointTorque[i][0] = dist(rng);
    }
  }
  robot.mbc().q[0] = {std::begin(rm->_default_attitude), std::end(rm->_default_attitude)};
  auto enc = std::vector<double>{};
  enc.reserve(robot.refJointOrder().size());
  auto vel = std::vector<double>{};
  vel.reserve(robot.refJointOrder().size());
  for(unsigned i = 0; i < robot.refJointOrder().size(); ++i)
  {
    enc.push_back(dist(rng));
    vel.push_back(dist(rng));
  }
  robot.data()->encoderValues = enc;
  robot.data()->encoderVelocities = vel;
  robot.forwardKinematics();

  robot.data()->forceSensors[robot.data()->forceSensorsIndex.at("RightFootForceSensor")].wrench(sva::ForceVecd{
      Eigen::Vector3d{dist(rng), dist(rng), dist(rng)}, Eigen::Vector3d{dist(rng), dist(rng), dist(rng)}});

  {
    auto converter = mc_rbdyn::RobotConverter{// XXX it would be better to write a fixture to check for all combinations
                                              // of these parameters
                                              robot, canonicalRobot, mc_rbdyn::RobotConverterConfig{}};
    for(unsigned i = 0; i < robot.mbc().q.size(); ++i)
    {
      if(robot.mbc().q.size() == 1) { BOOST_REQUIRE(robot.mbc().q[i][0] != 0); }
    }
    for(const auto & j : canonicalRobot.mb().joints())
    {
      auto cIdx = canonicalRobot.jointIndexByName(j.name());
      if(canonicalRobot.mbc().q[cIdx].size() != 1) continue;
      if(robot.hasJoint(j.name()))
      {
        auto idx = robot.jointIndexByName(j.name());
        if(j.isMimic())
        {
          auto mainIndex = canonicalRobot.jointIndexByName(j.mimicName());
          auto v = j.mimicMultiplier() * canonicalRobot.mbc().q[mainIndex][0] + j.mimicOffset();
          BOOST_REQUIRE(canonicalRobot.mbc().q[cIdx][0] == v);
        }
        else if(canonicalRobot.mbc().q[cIdx].size() == 1)
        {
          BOOST_REQUIRE(robot.mbc().q[idx][0] == canonicalRobot.mbc().q[cIdx][0]);
        }
      }
    }
    auto & crfs = canonicalRobot.forceSensor("RightFootForceSensor").force();
    auto & rfs = robot.forceSensor("RightFootForceSensor").force();
    BOOST_REQUIRE_CLOSE(rfs.x(), crfs.x(), 1e-10);
    BOOST_REQUIRE_CLOSE(rfs.x(), crfs.x(), 1e-10);
    BOOST_REQUIRE_CLOSE(rfs.x(), crfs.x(), 1e-10);
    for(unsigned i = 0; i < robot.refJointOrder().size(); ++i)
    {
      BOOST_REQUIRE(canonicalRobot.encoderValues()[i] == robot.encoderValues()[i]);
    }
  }
}
