#include <boost/test/unit_test.hpp>
#include <boost/mpl/list.hpp>

#include <mc_solver/ConstraintSetLoader.h>

#include <mc_solver/BoundedSpeedConstr.h>
#include <mc_solver/CollisionsConstraint.h>

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/configuration_io.h>

#include "utils.h"

static bool configured = configureRobotLoader();
/* Create Robots with one robot and an environment for the purpose of the test */
static auto rm = mc_rbdyn::RobotLoader::get_robot_module("HRP2DRC");
static auto em = mc_rbdyn::RobotLoader::get_robot_module("env",
                                                  std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH),
                                                  std::string("ground"));
static auto robots = mc_rbdyn::loadRobotAndEnv(*rm, *em);
static mc_solver::QPSolver solver(robots, 0.005);

template<typename T>
struct fail : public std::false_type {};

template<typename T>
struct ConstraintTester
{
  static_assert(fail<T>::value, "This should be specialized");
  mc_solver::ConstraintSetPtr make_ref() { return nullptr; }

  std::string json() { return ""; }

  void check(const mc_solver::ConstraintSetPtr & /*ref*/,
             const mc_solver::ConstraintSetPtr & /*loaded*/) {}
};

template<>
struct ConstraintTester<mc_solver::BoundedSpeedConstr>
{
  ConstraintTester<mc_solver::BoundedSpeedConstr>()
  {
    for(size_t i = 0; i < 3; ++i)
    {
      if(lS(i) > 0) { lS(i) = -lS(i); }
      uS(i) = -lS(i);
    }
  }

  mc_solver::ConstraintSetPtr make_ref()
  {
    auto ret = std::make_shared<mc_solver::BoundedSpeedConstr>(*robots, 0, solver.dt());
    ret->addBoundedSpeed(solver, "RARM_LINK6", Eigen::Vector3d::Zero(), Eigen::Matrix6d::Identity(), s);
    ret->addBoundedSpeed(solver, "LARM_LINK6", Eigen::Vector3d::Zero(), Eigen::Matrix6d::Identity(), lS, uS);
    return ret;
  }

  std::string json()
  {
    mc_rtc::Configuration config;
    config.add("type", "boundedSpeed");
    config.add("robotIndex", 0);
    auto cs = config.array("constraints");
    {
      mc_rtc::Configuration c;
      c.add("body", "RARM_LINK6");
      c.add("speed", s);
      cs.push(c);
    }
    {
      mc_rtc::Configuration c;
      c.add("body", "LARM_LINK6");
      c.add("lowerSpeed", lS);
      c.add("upperSpeed", lS);
      cs.push(c);
    }
    {
      // No speed entry so shouldn't matter
      mc_rtc::Configuration c;
      c.add("body", "RARM_LINK6");
      cs.push(c);
    }
    auto ret = getTmpFile();
    config.save(ret);
    return ret;
  }

  void check(const mc_solver::ConstraintSetPtr & ref_p,
             const mc_solver::ConstraintSetPtr & loaded_p)
  {
    auto ref = std::dynamic_pointer_cast<mc_solver::BoundedSpeedConstr>(ref_p);
    auto loaded = std::dynamic_pointer_cast<mc_solver::BoundedSpeedConstr>(loaded_p);
    BOOST_REQUIRE(ref);
    BOOST_REQUIRE(loaded);
    BOOST_CHECK(ref->nrBoundedSpeeds() == loaded->nrBoundedSpeeds());
    BOOST_CHECK(loaded->removeBoundedSpeed(solver, "RARM_LINK6"));
    BOOST_CHECK(loaded->removeBoundedSpeed(solver, "LARM_LINK6"));
  }

  Eigen::Vector3d s = Eigen::Vector3d::Random();
  Eigen::Vector3d lS = Eigen::Vector3d::Random();
  Eigen::Vector3d uS = Eigen::Vector3d::Zero();
};

template<>
struct ConstraintTester<mc_solver::CollisionsConstraint>
{
  mc_solver::ConstraintSetPtr make_ref()
  {
    auto ret = std::make_shared<mc_solver::CollisionsConstraint>(*robots, 0, 0, solver.dt());
    BOOST_REQUIRE(rm->commonSelfCollisions().size() > 0);
    ret->addCollisions(solver, rm->commonSelfCollisions());
    return ret;
  }

  std::string json()
  {
    mc_rtc::Configuration config;
    config.add("type", "collision");
    config.add("r1Index", 0);
    config.add("r2Index", 0);
    auto cv = config.array("collisions");
    for(const auto & c : rm->commonSelfCollisions())
    {
      cv.push(c);
    }
    auto ret = getTmpFile();
    config.save(ret);
    return ret;
  }

  void check(const mc_solver::ConstraintSetPtr & ref_p,
             const mc_solver::ConstraintSetPtr & loaded_p)
  {
    auto ref = std::dynamic_pointer_cast<mc_solver::CollisionsConstraint>(ref_p);
    auto loaded = std::dynamic_pointer_cast<mc_solver::CollisionsConstraint>(loaded_p);
    BOOST_REQUIRE(ref);
    BOOST_REQUIRE(loaded);
    BOOST_CHECK(ref->r1Index == loaded->r1Index);
    BOOST_CHECK(ref->r2Index == loaded->r2Index);
    BOOST_CHECK(ref->cols == loaded->cols);
  }
};

typedef boost::mpl::list<
          mc_solver::BoundedSpeedConstr,
          mc_solver::CollisionsConstraint> test_types;

BOOST_AUTO_TEST_CASE_TEMPLATE(TestConstraintSetLoader, T, test_types)
{
  auto tester = ConstraintTester<T>();
  auto ref = tester.make_ref();
  auto conf = tester.json();
  auto loaded = mc_solver::ConstraintSetLoader::load(solver, conf);
  tester.check(ref, loaded);
}
