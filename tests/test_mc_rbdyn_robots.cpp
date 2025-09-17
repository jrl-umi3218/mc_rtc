#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/rpy_utils.h>
#include <boost/test/unit_test.hpp>
#include "utils.h"
#include <chrono>
#include <random>

#include <sch/S_Object/S_Sphere.h>

mc_rbdyn::Robots & get_robots()
{
  static mc_rbdyn::RobotsPtr robots_ptr = nullptr;
  if(robots_ptr) { return *robots_ptr; }
  configureRobotLoader();
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
  auto env = mc_rbdyn::RobotLoader::get_robot_module("env", std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH),
                                                     std::string("ground"));
  robots_ptr = mc_rbdyn::loadRobots({rm, env});
  return *robots_ptr;
}

void TestRobotLoadingCommon(mc_rbdyn::RobotModulePtr rm, mc_rbdyn::RobotModulePtr envrm)
{
  // Non-unique names
  BOOST_REQUIRE_THROW(mc_rbdyn::loadRobots({rm, rm}), std::runtime_error);
  mc_rbdyn::RobotsPtr robots_ptr = nullptr;
  BOOST_REQUIRE_NO_THROW(robots_ptr = mc_rbdyn::loadRobots({rm, envrm}));
  BOOST_REQUIRE(robots_ptr->hasRobot(rm->name));
  BOOST_REQUIRE(robots_ptr->hasRobot(envrm->name));
  {
    auto & robot = robots_ptr->robot(rm->name);
    auto & env = robots_ptr->robot(envrm->name);
    BOOST_REQUIRE_EQUAL(robot.name(), rm->name);
    BOOST_REQUIRE_EQUAL(robot.robotIndex(), 0);
    BOOST_REQUIRE_EQUAL(env.name(), envrm->name);
    BOOST_REQUIRE_EQUAL(env.robotIndex(), 1);
    robots_ptr->rename(robot.name(), "renamed");
    BOOST_REQUIRE(robots_ptr->hasRobot("renamed"));
    auto & renamed = robots_ptr->robot("renamed");
    BOOST_REQUIRE_EQUAL(renamed.name(), "renamed");
    BOOST_REQUIRE_EQUAL(robot.name(), "renamed");
    BOOST_REQUIRE_EQUAL(renamed.robotIndex(), 0);
    BOOST_REQUIRE_EQUAL(robot.robotIndex(), 0);
    BOOST_REQUIRE(robots_ptr->hasRobot(envrm->name));
    BOOST_REQUIRE_EQUAL(robots_ptr->robot(envrm->name).name(), envrm->name);
    BOOST_REQUIRE_EQUAL(robots_ptr->robot(envrm->name).robotIndex(), 1);
    BOOST_REQUIRE_NO_THROW(robots_ptr->robotCopy(robot, "robotCopy"));
  }

  BOOST_REQUIRE(robots_ptr->hasRobot("robotCopy"));
  auto & robotCopy = robots_ptr->robot(robots_ptr->size() - 1);
  BOOST_REQUIRE(robotCopy.name() != rm->name);
  BOOST_REQUIRE_EQUAL(robots_ptr->robot("robotCopy").name(), "robotCopy");
  BOOST_REQUIRE_EQUAL(robotCopy.robotIndex(), 2);
  BOOST_REQUIRE_EQUAL(robotCopy.name(), "robotCopy");
  auto & robot = robots_ptr->robot("renamed");
  for(const auto & c : robot.convexes()) { BOOST_REQUIRE(robotCopy.hasConvex(c.first)); }
  for(const auto & s : robot.surfaces()) { BOOST_REQUIRE(robotCopy.hasSurface(s.first)); }
  for(const auto & fs : robot.forceSensors()) { BOOST_REQUIRE(robotCopy.hasForceSensor(fs.name())); }
  for(const auto & bs : robot.bodySensors()) { BOOST_REQUIRE(robotCopy.hasBodySensor(bs.name())); }

  robots_ptr->removeRobot("robotCopy");
  BOOST_REQUIRE(!robots_ptr->hasRobot("robotCopy"));
  BOOST_REQUIRE(robots_ptr->hasRobot("renamed"));
  BOOST_REQUIRE(robots_ptr->hasRobot(envrm->name));
}

BOOST_AUTO_TEST_CASE(TestRobotLoading)
{
  configureRobotLoader();
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
  auto envrm = mc_rbdyn::RobotLoader::get_robot_module("env", std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH),
                                                       std::string("ground"));
  TestRobotLoadingCommon(rm, envrm);
}

BOOST_AUTO_TEST_CASE(TestRobotLoadingWithCollisionObjects)
{
  configureRobotLoader();
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
  rm->_collisionObjects["L_HAND_SPHERE"] = {"L_WRIST_Y_S", std::make_shared<sch::S_Sphere>(0.09)};
  rm->_collisionTransforms["L_HAND_SPHERE"] = sva::PTransformd::Identity();
  auto envrm = mc_rbdyn::RobotLoader::get_robot_module("env", std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH),
                                                       std::string("ground"));
  TestRobotLoadingCommon(rm, envrm);
}

BOOST_AUTO_TEST_CASE(TestRobotPosWVelWAccW)
{
  auto & robots = get_robots();

  for(int i = 0; i < 100; ++i)
  {
    Eigen::Vector3d oriRPYRef = Eigen::Vector3d::Random();
    Eigen::Vector3d posRPYRef = Eigen::Vector3d::Random();
    sva::PTransformd refPosW(mc_rbdyn::rpyToMat(oriRPYRef), posRPYRef);
    robots.robot().posW(refPosW);
    BOOST_CHECK(robots.robot().posW().matrix().isApprox(refPosW.matrix()));
  }

  auto checkVelocity = [](const sva::MotionVecd & actual, const sva::MotionVecd & refVal)
  {
    BOOST_CHECK_MESSAGE(actual.vector().isApprox(refVal.vector()),
                        "Error in Robot::velW" << "\nExpected:" << "\nangular:" << refVal.angular().transpose()
                                               << "\nlinear :" << refVal.linear().transpose()
                                               << "\nGot:" << "\nangular:" << actual.angular().transpose()
                                               << "\nlinear :" << actual.linear().transpose());
  };

  for(int i = 0; i < 100; ++i)
  {
    auto refVal = sva::MotionVecd{Eigen::Vector3d::Random(), Eigen::Vector3d::Random()};
    robots.robot().velW(refVal);
    robots.robot().accW(refVal);
    checkVelocity(robots.robot().velW(), refVal);
    checkVelocity(robots.robot().accW(), refVal);
  }
}

BOOST_AUTO_TEST_CASE(TestRobotZMPSimple)
{
  auto & robots = get_robots();
  auto & robot = robots.robot();

  // Put all mass on the left foot, ZMP should be under the sensor
  const auto normalForce = robot.mass() * 10;
  const auto sensorNames = std::vector<std::string>{"LeftFootForceSensor", "RightFootForceSensor"};
  auto & lfs = robot.data()->forceSensors[robot.data()->forceSensorsIndex.at("LeftFootForceSensor")];
  auto & rfs = robot.data()->forceSensors[robot.data()->forceSensorsIndex.at("RightFootForceSensor")];
  // Prevent using the JVRC1 calibration files (when they exist)
  // for the ZMP computation to obtain repeatable results here.
  // Resetting the calibrator makes sure that it has no effect.
  //
  // This test assumes that the sensor is at the model position and that there
  // is no force offset.
  //
  // XXX We should investigate the effect of calibrator on ZMP measurement,
  // and write a test that checks this case as well
  lfs.resetCalibrator();
  rfs.resetCalibrator();

  {
    // ZMP under left sensor
    const auto forceLeftSurface = sva::ForceVecd{Eigen::Vector3d::Zero(), {0., 0., normalForce}};
    sva::PTransformd X_0_ls = lfs.X_0_f(robot);
    X_0_ls.translation().z() = 0;
    auto X_ls_f = lfs.X_0_f(robot) * X_0_ls.inv();
    lfs.wrench(X_ls_f.dualMul(forceLeftSurface));
    rfs.wrench(sva::ForceVecd::Zero());

    auto zmpIdeal = X_0_ls.translation();
    auto zmpComputed = robot.zmp(sensorNames, Eigen::Vector3d::Zero(), {0., 0., 1.});
    BOOST_CHECK_MESSAGE(zmpComputed.isApprox(zmpIdeal, 1e-10),
                        "Error in Robot::zmp computation with leftFootRatio=" << "\nExpected: " << zmpIdeal.transpose()
                                                                              << "\nGot: " << zmpComputed.transpose());
  }

  {
    // ZMP under right sensor
    const auto forceRightSurface = sva::ForceVecd{Eigen::Vector3d::Zero(), {0., 0., normalForce}};
    sva::PTransformd X_0_rs = rfs.X_0_f(robot);
    X_0_rs.translation().z() = 0;
    auto X_rs_f = lfs.X_0_f(robot) * X_0_rs.inv();
    lfs.wrench(X_rs_f.dualMul(forceRightSurface));
    rfs.wrench(sva::ForceVecd::Zero());

    auto zmpIdeal = X_0_rs.translation();
    auto zmpComputed = robot.zmp(sensorNames, Eigen::Vector3d::Zero(), {0., 0., 1.});
    BOOST_CHECK_MESSAGE(zmpComputed.isApprox(zmpIdeal, 1e-10),
                        "Error in Robot::zmp computation with leftFootRatio=" << "\nExpected: " << zmpIdeal.transpose()
                                                                              << "\nGot: " << zmpComputed.transpose());
  }

  { // checks that zmp throws if used with null force
    rfs.wrench(sva::ForceVecd::Zero());
    lfs.wrench(sva::ForceVecd::Zero());
    BOOST_CHECK_THROW(robot.zmp(sensorNames, Eigen::Vector3d::Zero(), {0., 0., 1.}), std::runtime_error);
  }
}
