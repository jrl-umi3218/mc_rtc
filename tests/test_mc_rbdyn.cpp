#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/rpy_utils.h>
#include <boost/test/unit_test.hpp>
#include "utils.h"
#include <chrono>
#include <random>

mc_rbdyn::Robots & get_robots()
{
  static std::shared_ptr<mc_rbdyn::Robots> robots_ptr = nullptr;
  if(robots_ptr)
  {
    return *robots_ptr;
  }
  configureRobotLoader();
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
  auto env = mc_rbdyn::RobotLoader::get_robot_module("env", std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH),
                                                     std::string("ground"));
  robots_ptr = mc_rbdyn::loadRobots({rm, env});
  return *robots_ptr;
}

BOOST_AUTO_TEST_CASE(TestRobotPosWVelWAccW)
{
  auto robots = get_robots();

  for(int i = 0; i < 100; ++i)
  {
    Eigen::Vector3d oriRPYRef = Eigen::Vector3d::Random();
    Eigen::Vector3d posRPYRef = Eigen::Vector3d::Random();
    sva::PTransformd refPosW(mc_rbdyn::rpyToMat(oriRPYRef), posRPYRef);
    robots.robot().posW(refPosW);
    BOOST_CHECK(robots.robot().posW().matrix().isApprox(refPosW.matrix()));
  }

  auto checkVelocity = [](const sva::MotionVecd & actual, const sva::MotionVecd & refVal) {
    BOOST_CHECK_MESSAGE(actual.vector().isApprox(refVal.vector()), "Error in Robot::velW"
                                                                       << "\nExpected:"
                                                                       << "\nangular:" << refVal.angular().transpose()
                                                                       << "\nlinear :" << refVal.linear().transpose()
                                                                       << "\nGot:"
                                                                       << "\nangular:" << actual.angular().transpose()
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
  auto robots = get_robots();
  auto & robot = robots.robot();

  // Put all mass on the left foot, ZMP should be under the sensor
  const auto normalForce = robot.mass() * 10;
  const auto sensorNames = std::vector<std::string>{"LeftFootForceSensor", "RightFootForceSensor"};
  auto & lfs = robot.forceSensor("LeftFootForceSensor");
  auto & rfs = robot.forceSensor("RightFootForceSensor");

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
    BOOST_CHECK_MESSAGE(zmpComputed.isApprox(zmpIdeal, 1e-10), "Error in Robot::zmp computation with leftFootRatio="
                                                                   << "\nExpected: " << zmpIdeal.transpose()
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
    BOOST_CHECK_MESSAGE(zmpComputed.isApprox(zmpIdeal, 1e-10), "Error in Robot::zmp computation with leftFootRatio="
                                                                   << "\nExpected: " << zmpIdeal.transpose()
                                                                   << "\nGot: " << zmpComputed.transpose());
  }

  { // checks that zmp throws if used with null force
    rfs.wrench(sva::ForceVecd::Zero());
    lfs.wrench(sva::ForceVecd::Zero());
    BOOST_CHECK_THROW(robot.zmp(sensorNames, Eigen::Vector3d::Zero(), {0., 0., 1.}), std::runtime_error);
  }
}
