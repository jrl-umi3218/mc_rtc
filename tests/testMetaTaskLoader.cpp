#include <boost/test/unit_test.hpp>
#include <boost/mpl/list.hpp>

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_tasks/AddRemoveContactTask.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/ComplianceTask.h>

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
struct TaskTester
{
  static_assert(fail<T>::value, "This should be specialized");
  mc_tasks::MetaTaskPtr make_ref() { return nullptr; }

  std::string json() { return ""; }

  void check(const mc_tasks::MetaTaskPtr & /*ref*/,
             const mc_tasks::MetaTaskPtr & /*loaded*/) {}
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

#define AddRemoveContactTaskTester(T, typeStr) \
template<> \
struct TaskTester<T> \
{ \
  mc_tasks::MetaTaskPtr make_ref() \
  { \
    auto ret  = std::make_shared<T>(solver, contact, speed, stiffness, weight); \
    return ret; \
  } \
 \
  std::string json() \
  { \
    mc_rtc::Configuration config; \
    config.add("type", typeStr); \
    config.add("contact", contact); \
    config.add("stiffness", stiffness); \
    config.add("weight", weight); \
    config.add("speed", speed); \
    auto ret = getTmpFile(); \
    config.save(ret); \
    return ret; \
  } \
 \
  void check(const mc_tasks::MetaTaskPtr & ref_p, \
             const mc_tasks::MetaTaskPtr & loaded_p) \
  { \
    auto ref = std::dynamic_pointer_cast<T>(ref_p); \
    auto loaded = std::dynamic_pointer_cast<T>(loaded_p); \
    BOOST_REQUIRE(ref); \
    BOOST_REQUIRE(loaded); \
    BOOST_CHECK_CLOSE(ref->stiffness(), loaded->stiffness(), 1e-6); \
    BOOST_CHECK_CLOSE(ref->weight(), loaded->weight(), 1e-6); \
    BOOST_CHECK_CLOSE(ref->speed(), loaded->speed(), 1e-6); \
    auto ref_surf = ref->robotSurf; \
    auto loaded_surf = loaded->robotSurf; \
    BOOST_CHECK(ref_surf->type() == loaded_surf->type()); \
    BOOST_CHECK(ref_surf->name() == loaded_surf->name()); \
    BOOST_CHECK(ref->bodyId == loaded->bodyId); \
    BOOST_CHECK(ref->robotBodyIndex == loaded->robotBodyIndex); \
    BOOST_CHECK(ref->targetTf == loaded->targetTf); \
  } \
 \
  mc_rbdyn::Contact contact = mc_rbdyn::Contact(*robots, "LFullSole", "AllGround"); \
  double speed = fabs(rnd()); \
  double stiffness = fabs(rnd()); \
  double weight = fabs(rnd()); \
};
AddRemoveContactTaskTester(mc_tasks::AddContactTask, "addContact")
AddRemoveContactTaskTester(mc_tasks::RemoveContactTask, "removeContact")

template<>
struct TaskTester<mc_tasks::ComplianceTask>
{
  mc_tasks::MetaTaskPtr make_ref()
  {
    auto t = std::shared_ptr<mc_tasks::ComplianceTask>(new mc_tasks::ComplianceTask(*robots, 0, "RARM_LINK6", solver.dt(), dof, stiffness, weight, forceThresh, torqueThresh, forceGain, torqueGain));
    t->setTargetWrench(wrench);
    return t;
  }

  std::string json()
  {
    mc_rtc::Configuration config;
    config.add("type", "compliance");
    config.add("robotIndex", 0);
    config.add("body", "RARM_LINK6");
    config.add("dof", dof);
    config.add("stiffness", stiffness);
    config.add("weight", weight);
    config.add("forceThresh", forceThresh);
    config.add("torqueThresh", torqueThresh);
    config.add("forceGain", forceGain);
    config.add("torqueGain", torqueGain);
    config.add("wrench", wrench);
    auto ret = getTmpFile();
    config.save(ret);
    return ret;
  }

  void check(const mc_tasks::MetaTaskPtr & ref_p,
             const mc_tasks::MetaTaskPtr & loaded_p)
  {
    auto ref = std::dynamic_pointer_cast<mc_tasks::ComplianceTask>(ref_p);
    auto loaded = std::dynamic_pointer_cast<mc_tasks::ComplianceTask>(loaded_p);
    BOOST_REQUIRE(ref);
    BOOST_REQUIRE(loaded);
    BOOST_CHECK(ref->dof().isApprox(loaded->dof(), 1e-6));
    BOOST_CHECK_CLOSE(ref->stiffness(), loaded->stiffness(), 1e-6);
    BOOST_CHECK_CLOSE(ref->weight(), loaded->weight(), 1e-6);
    BOOST_CHECK_CLOSE(ref->forceThresh(), loaded->forceThresh(), 1e-6);
    BOOST_CHECK_CLOSE(ref->torqueThresh(), loaded->torqueThresh(), 1e-6);
    BOOST_CHECK_CLOSE(ref->forceGain().first, loaded->forceGain().first, 1e-6);
    BOOST_CHECK_CLOSE(ref->forceGain().second, loaded->forceGain().second, 1e-6);
    BOOST_CHECK_CLOSE(ref->torqueGain().first, loaded->torqueGain().first, 1e-6);
    BOOST_CHECK_CLOSE(ref->torqueGain().second, loaded->torqueGain().second, 1e-6);
    BOOST_CHECK(ref->getTargetWrench() == loaded->getTargetWrench());
  }

  Eigen::Matrix6d dof = Eigen::Matrix6d::Random();
  double stiffness = fabs(rnd());
  double weight = fabs(rnd());
  double forceThresh = fabs(rnd());
  double torqueThresh = fabs(rnd());
  std::pair<double, double> forceGain = {fabs(rnd()), fabs(rnd())};
  std::pair<double, double> torqueGain = {fabs(rnd()), fabs(rnd())};
  sva::ForceVecd wrench = {Eigen::Vector6d::Random()};
};

typedef boost::mpl::list<mc_tasks::CoMTask,
                         mc_tasks::AddContactTask,
                         mc_tasks::RemoveContactTask,
                         mc_tasks::ComplianceTask> test_types;

BOOST_AUTO_TEST_CASE_TEMPLATE(TestMetaTaskLoader, T, test_types)
{
  auto tester = TaskTester<T>();
  auto ref = tester.make_ref();
  auto conf = tester.json();
  auto loaded = mc_tasks::MetaTaskLoader::load(solver, conf);
  tester.check(ref, loaded);
}
