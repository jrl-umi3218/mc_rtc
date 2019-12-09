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

BOOST_AUTO_TEST_CASE(TestRobotPosWVelW)
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

  for(int i = 0; i < 100; ++i)
  {
    auto refVal = sva::MotionVecd{Eigen::Vector3d::Random(), Eigen::Vector3d::Random()};
    robots.robot().velW(refVal);
    auto actual = robots.robot().velW();
    BOOST_CHECK_MESSAGE(actual.vector().isApprox(refVal.vector()), "Error in Robot::velW"
                                                                       << "\nExpected:"
                                                                       << "\nangular:" << refVal.angular().transpose()
                                                                       << "\nlinear :" << refVal.linear().transpose()
                                                                       << "\nGot:"
                                                                       << "\nangular:" << actual.angular().transpose()
                                                                       << "\nlinear :" << actual.linear().transpose());
  }
}

BOOST_AUTO_TEST_CASE(TestRobotZMP)
{
  auto robots = get_robots();
  auto & robot = robots.robot();

  // Put all mass on the left foot, ZMP should be under the sensor
  const auto normalForce = robot.mass() * 10;
  const auto sensorNames = std::vector<std::string>{"LeftFootForceSensor", "RightFootForceSensor"};
  auto & lfs = robot.forceSensor("LeftFootForceSensor");
  auto & rfs = robot.forceSensor("RightFootForceSensor");

  // see https://en.cppreference.com/w/cpp/numeric/random/uniform_real_distribution
  std::mt19937_64 rng;
  // initialize the random number generator with time-dependent seed
  auto timeSeed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
  std::seed_seq ss{uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed >> 32)};
  rng.seed(ss);
  // initialize a uniform distribution between 0 and 1
  std::uniform_real_distribution<double> unif(0, 1);
  std::uniform_real_distribution<double> unifNeg(-1, 1);

  constexpr double xFootLength = 0.1;

  // Test a random ZMP defined:
  // - Anywhere between LeftFootCenter and RightFootCenter
  // - At a random position within the foot surface in x (assuming foot length
  // xFootLength)
  auto testZMP = [&]() {
    double x = xFootLength * unifNeg(rng);
    double leftFootRatio = unif(rng);

    const auto forceLeftSurface = sva::ForceVecd{Eigen::Vector3d::Zero(), {0., 0., (1 - leftFootRatio) * normalForce}};
    const auto forceRightSurface = sva::ForceVecd{Eigen::Vector3d::Zero(), {0., 0., leftFootRatio * normalForce}};

    // Put the weight in-between the left and right foot surfaces along a line
    // offset by x from the foot center
    auto X_offset = sva::PTransformd{Eigen::Vector3d{x, 0, 0}};
    auto X_0_ls = X_offset * robot.surfacePose("LeftFootCenter");
    auto X_ls_f = lfs.X_0_f(robot) * X_0_ls.inv();
    lfs.wrench(X_ls_f.dualMul(forceLeftSurface));

    auto X_0_rs = X_offset * robot.surfacePose("LeftFootCenter");
    auto X_rs_f = rfs.X_0_f(robot) * X_0_rs.inv();
    rfs.wrench(X_rs_f.dualMul(forceRightSurface));

    Eigen::Vector3d zmpIdeal = sva::interpolate(X_0_rs, X_0_ls, leftFootRatio).translation();
    zmpIdeal.z() = 0;

    auto zmpComputed = robot.zmp(sensorNames, Eigen::Vector3d::Zero(), {0., 0., 1.});
    BOOST_CHECK_MESSAGE(zmpComputed.isApprox(zmpIdeal, 10e-4),
                        "Error in Robot::zmp computation with leftFootRatio=" << leftFootRatio
                                                                              << "\nExpected: " << zmpIdeal.transpose()
                                                                              << "\nGot: " << zmpComputed.transpose());
  };

  // Tests 100 random ZMP with vertical force applied in-between the feet
  // surfaces
  for(int i = 0; i < 100; ++i)
  {
    testZMP();
  }
}
