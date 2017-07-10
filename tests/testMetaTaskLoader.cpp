#include <boost/test/unit_test.hpp>
#include <boost/mpl/list.hpp>

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_tasks/CoMTask.h>

#include <mc_rbdyn/RobotLoader.h>

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
struct TaskTester
{
  mc_tasks::MetaTaskPtr make_ref()
  {
    static_assert(fail<T>::value, "This should be specialized");
    return nullptr;
  }

  std::string json()
  {
    static_assert(fail<T>::value, "This should be specialized");
    return "";
  }

  void check(const mc_tasks::MetaTaskPtr & /*ref*/,
             const mc_tasks::MetaTaskPtr & /*loaded*/)
  {
    static_assert(fail<T>::value, "This should be specialized");
  }
};

template<>
struct TaskTester<mc_tasks::CoMTask>
{
  mc_tasks::MetaTaskPtr make_ref()
  {
    auto ret = std::make_shared<mc_tasks::CoMTask>(*robots, 0, stiffness, weight);
    ret->com(com);
    return ret;
  }

  std::string json()
  {
    mc_rtc::Configuration config;
    config.add("type", "com");
    config.add("robotIndex", 0);
    config.add("stiffness", stiffness);
    config.add("weight", weight);
    config.add("com", com);
    auto ret = getTmpFile();
    config.save(ret);
    return ret;
  }

  void check(const mc_tasks::MetaTaskPtr & ref,
             const mc_tasks::MetaTaskPtr & loaded)
  {
    auto com_ref = std::dynamic_pointer_cast<mc_tasks::CoMTask>(ref);
    auto com_loaded = std::dynamic_pointer_cast<mc_tasks::CoMTask>(loaded);
    BOOST_REQUIRE(com_ref);
    BOOST_REQUIRE(com_loaded);
    BOOST_CHECK_CLOSE(com_ref->stiffness(), com_loaded->stiffness(), 1e-6);
    BOOST_CHECK_CLOSE(com_ref->weight(), com_loaded->weight(), 1e-6);
    BOOST_CHECK(com_ref->com().isApprox(com_loaded->com(), 1e-9));
  }

  Eigen::Vector3d com = Eigen::Vector3d::Random();
  double stiffness = fabs(rnd());
  double weight = fabs(rnd());
};

typedef boost::mpl::list<mc_tasks::CoMTask> test_types;

BOOST_AUTO_TEST_CASE_TEMPLATE(TestMetaTaskLoader, T, test_types)
{
  auto tester = TaskTester<T>();
  auto ref = tester.make_ref();
  auto conf = tester.json();
  auto loaded = mc_tasks::MetaTaskLoader::load(solver, conf);
  tester.check(ref, loaded);
}
