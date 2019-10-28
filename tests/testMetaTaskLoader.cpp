/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/configuration_io.h>
#include <mc_tasks/AddRemoveContactTask.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/ComplianceTask.h>
#include <mc_tasks/GazeTask.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/PositionBasedVisServoTask.h>
#include <mc_tasks/RelativeEndEffectorTask.h>
#include <mc_tasks/SurfaceTransformTask.h>
#include <mc_tasks/VectorOrientationTask.h>

#include <boost/mpl/list.hpp>
#include <boost/test/unit_test.hpp>

#include "utils.h"

static bool configured = configureRobotLoader();
/* Create Robots with one robot and an environment for the purpose of the test */
static auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
static auto em =
    mc_rbdyn::RobotLoader::get_robot_module("env", std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH), std::string("ground"));
static auto robots = mc_rbdyn::loadRobotAndEnv(*rm, *em);
static mc_solver::QPSolver solver(robots, 0.005);

template<typename T>
struct fail : public std::false_type
{
};

template<typename T>
struct TaskTester
{
  static_assert(fail<T>::value, "This should be specialized");
  mc_tasks::MetaTaskPtr make_ref()
  {
    return nullptr;
  }

  std::string json()
  {
    return "";
  }

  void check(const mc_tasks::MetaTaskPtr & /*ref*/, const mc_tasks::MetaTaskPtr & /*loaded*/) {}
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

  void check(const mc_tasks::MetaTaskPtr & ref_p, const mc_tasks::MetaTaskPtr & loaded_p)
  {
    auto ref = std::dynamic_pointer_cast<mc_tasks::CoMTask>(ref_p);
    auto loaded = std::dynamic_pointer_cast<mc_tasks::CoMTask>(loaded_p);
    BOOST_REQUIRE(ref);
    BOOST_REQUIRE(loaded);
    BOOST_CHECK_CLOSE(ref->stiffness(), loaded->stiffness(), 1e-6);
    BOOST_CHECK_CLOSE(ref->weight(), loaded->weight(), 1e-6);
    BOOST_CHECK(ref->com().isApprox(loaded->com(), 1e-9));
  }

  Eigen::Vector3d com = Eigen::Vector3d::Random();
  double stiffness = fabs(rnd());
  double weight = fabs(rnd());
};

#define AddRemoveContactTaskTester(T, typeStr)                                              \
  template<>                                                                                \
  struct TaskTester<T>                                                                      \
  {                                                                                         \
    mc_tasks::MetaTaskPtr make_ref()                                                        \
    {                                                                                       \
      auto ret = std::make_shared<T>(solver, contact, speed, stiffness, weight);            \
      return ret;                                                                           \
    }                                                                                       \
                                                                                            \
    std::string json()                                                                      \
    {                                                                                       \
      mc_rtc::Configuration config;                                                         \
      config.add("type", typeStr);                                                          \
      config.add("contact", contact);                                                       \
      config.add("stiffness", stiffness);                                                   \
      config.add("weight", weight);                                                         \
      config.add("speed", speed);                                                           \
      auto ret = getTmpFile();                                                              \
      config.save(ret);                                                                     \
      return ret;                                                                           \
    }                                                                                       \
                                                                                            \
    void check(const mc_tasks::MetaTaskPtr & ref_p, const mc_tasks::MetaTaskPtr & loaded_p) \
    {                                                                                       \
      auto ref = std::dynamic_pointer_cast<T>(ref_p);                                       \
      auto loaded = std::dynamic_pointer_cast<T>(loaded_p);                                 \
      BOOST_REQUIRE(ref);                                                                   \
      BOOST_REQUIRE(loaded);                                                                \
      BOOST_CHECK_CLOSE(ref->stiffness(), loaded->stiffness(), 1e-6);                       \
      BOOST_CHECK_CLOSE(ref->weight(), loaded->weight(), 1e-6);                             \
      BOOST_CHECK_CLOSE(ref->speed(), loaded->speed(), 1e-6);                               \
      auto ref_surf = ref->robotSurf;                                                       \
      auto loaded_surf = loaded->robotSurf;                                                 \
      BOOST_CHECK(ref_surf->type() == loaded_surf->type());                                 \
      BOOST_CHECK(ref_surf->name() == loaded_surf->name());                                 \
      BOOST_CHECK(ref->bodyId == loaded->bodyId);                                           \
      BOOST_CHECK(ref->robotBodyIndex == loaded->robotBodyIndex);                           \
      BOOST_CHECK(ref->targetTf == loaded->targetTf);                                       \
    }                                                                                       \
                                                                                            \
    mc_rbdyn::Contact contact = mc_rbdyn::Contact(*robots, "LeftFoot", "AllGround");        \
    double speed = fabs(rnd());                                                             \
    double stiffness = fabs(rnd());                                                         \
    double weight = fabs(rnd());                                                            \
  };
AddRemoveContactTaskTester(mc_tasks::AddContactTask, "addContact")
    AddRemoveContactTaskTester(mc_tasks::RemoveContactTask, "removeContact")

        template<>
        struct TaskTester<mc_tasks::force::ComplianceTask>
{
  mc_tasks::MetaTaskPtr make_ref()
  {
    auto t = std::shared_ptr<mc_tasks::force::ComplianceTask>(
        new mc_tasks::force::ComplianceTask(*robots, 0, "R_WRIST_Y_S", solver.dt(), dof, stiffness, weight, forceThresh,
                                            torqueThresh, forceGain, torqueGain));
    t->setTargetWrench(wrench);
    return t;
  }

  std::string json()
  {
    mc_rtc::Configuration config;
    config.add("type", "compliance");
    config.add("robotIndex", 0);
    config.add("body", "R_WRIST_Y_S");
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

  void check(const mc_tasks::MetaTaskPtr & ref_p, const mc_tasks::MetaTaskPtr & loaded_p)
  {
    auto ref = std::dynamic_pointer_cast<mc_tasks::force::ComplianceTask>(ref_p);
    auto loaded = std::dynamic_pointer_cast<mc_tasks::force::ComplianceTask>(loaded_p);
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
    BOOST_CHECK(ref->getTargetWrench().vector().isApprox(loaded->getTargetWrench().vector()));
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

template<>
struct TaskTester<mc_tasks::OrientationTask>
{
  mc_tasks::MetaTaskPtr make_ref()
  {
    auto t = std::make_shared<mc_tasks::OrientationTask>("R_WRIST_Y_S", *robots, 0, stiffness, weight);
    t->orientation(ori);
    return t;
  }

  std::string json()
  {
    mc_rtc::Configuration config;
    config.add("type", "orientation");
    config.add("robotIndex", 0);
    config.add("body", "R_WRIST_Y_S");
    config.add("stiffness", stiffness);
    config.add("weight", weight);
    config.add("orientation", ori);
    auto ret = getTmpFile();
    config.save(ret);
    return ret;
  }

  void check(const mc_tasks::MetaTaskPtr & ref_p, const mc_tasks::MetaTaskPtr & loaded_p)
  {
    auto ref = std::dynamic_pointer_cast<mc_tasks::OrientationTask>(ref_p);
    auto loaded = std::dynamic_pointer_cast<mc_tasks::OrientationTask>(loaded_p);
    BOOST_REQUIRE(ref);
    BOOST_REQUIRE(loaded);
    BOOST_CHECK_CLOSE(ref->stiffness(), loaded->stiffness(), 1e-6);
    BOOST_CHECK_CLOSE(ref->weight(), loaded->weight(), 1e-6);
    BOOST_CHECK(ref->orientation().isApprox(loaded->orientation(), 1e-6));
  }

  double stiffness = fabs(rnd());
  double weight = fabs(rnd());
  Eigen::Matrix3d ori = Eigen::Matrix3d::Random();
};

template<>
struct TaskTester<mc_tasks::PositionTask>
{
  mc_tasks::MetaTaskPtr make_ref()
  {
    auto t = std::make_shared<mc_tasks::PositionTask>("R_WRIST_Y_S", *robots, 0, stiffness, weight);
    t->position(pos);
    return t;
  }

  std::string json()
  {
    mc_rtc::Configuration config;
    config.add("type", "position");
    config.add("robotIndex", 0);
    config.add("body", "R_WRIST_Y_S");
    config.add("stiffness", stiffness);
    config.add("weight", weight);
    config.add("position", pos);
    auto ret = getTmpFile();
    config.save(ret);
    return ret;
  }

  void check(const mc_tasks::MetaTaskPtr & ref_p, const mc_tasks::MetaTaskPtr & loaded_p)
  {
    auto ref = std::dynamic_pointer_cast<mc_tasks::PositionTask>(ref_p);
    auto loaded = std::dynamic_pointer_cast<mc_tasks::PositionTask>(loaded_p);
    BOOST_REQUIRE(ref);
    BOOST_REQUIRE(loaded);
    BOOST_CHECK_CLOSE(ref->stiffness(), loaded->stiffness(), 1e-6);
    BOOST_CHECK_CLOSE(ref->weight(), loaded->weight(), 1e-6);
    BOOST_CHECK(ref->position().isApprox(loaded->position(), 1e-6));
  }

  double stiffness = fabs(rnd());
  double weight = fabs(rnd());
  Eigen::Vector3d pos = Eigen::Vector3d::Random();
};

template<>
struct TaskTester<mc_tasks::EndEffectorTask>
{
  mc_tasks::MetaTaskPtr make_ref()
  {
    auto t = std::make_shared<mc_tasks::EndEffectorTask>("R_WRIST_Y_S", *robots, 0, stiffness, weight);
    t->set_ef_pose({ori, pos});
    return t;
  }

  std::string json()
  {
    mc_rtc::Configuration config;
    config.add("type", "body6d");
    config.add("robotIndex", 0);
    config.add("body", "R_WRIST_Y_S");
    config.add("stiffness", stiffness);
    config.add("weight", weight);
    config.add("position", pos);
    config.add("orientation", ori);
    auto ret = getTmpFile();
    config.save(ret);
    return ret;
  }

  void check(const mc_tasks::MetaTaskPtr & ref_p, const mc_tasks::MetaTaskPtr & loaded_p)
  {
    auto ref = std::dynamic_pointer_cast<mc_tasks::EndEffectorTask>(ref_p);
    auto loaded = std::dynamic_pointer_cast<mc_tasks::EndEffectorTask>(loaded_p);
    BOOST_REQUIRE(ref);
    BOOST_REQUIRE(loaded);
    BOOST_CHECK_CLOSE(ref->positionTask->stiffness(), loaded->positionTask->stiffness(), 1e-6);
    BOOST_CHECK_CLOSE(ref->positionTask->weight(), loaded->positionTask->weight(), 1e-6);
    BOOST_CHECK(ref->positionTask->position().isApprox(loaded->positionTask->position(), 1e-6));
    BOOST_CHECK_CLOSE(ref->orientationTask->stiffness(), loaded->orientationTask->stiffness(), 1e-6);
    BOOST_CHECK_CLOSE(ref->orientationTask->weight(), loaded->orientationTask->weight(), 1e-6);
    BOOST_CHECK(ref->orientationTask->orientation().isApprox(loaded->orientationTask->orientation(), 1e-6));
  }

  double stiffness = fabs(rnd());
  double weight = fabs(rnd());
  Eigen::Matrix3d ori = Eigen::Matrix3d::Random();
  Eigen::Vector3d pos = Eigen::Vector3d::Random();
};

template<>
struct TaskTester<mc_tasks::RelativeEndEffectorTask>
{
  mc_tasks::MetaTaskPtr make_ref()
  {
    auto t = std::make_shared<mc_tasks::RelativeEndEffectorTask>("R_WRIST_Y_S", *robots, 0, "L_WRIST_Y_S", stiffness,
                                                                 weight);
    t->set_ef_pose({ori, pos});
    return t;
  }

  std::string json()
  {
    mc_rtc::Configuration config;
    config.add("type", "relBody6d");
    config.add("robotIndex", 0);
    config.add("body", "R_WRIST_Y_S");
    config.add("relBody", "L_WRIST_Y_S");
    config.add("stiffness", stiffness);
    config.add("weight", weight);
    config.add("position", pos);
    config.add("orientation", ori);
    auto ret = getTmpFile();
    config.save(ret);
    return ret;
  }

  void check(const mc_tasks::MetaTaskPtr & ref_p, const mc_tasks::MetaTaskPtr & loaded_p)
  {
    auto ref = std::dynamic_pointer_cast<mc_tasks::RelativeEndEffectorTask>(ref_p);
    auto loaded = std::dynamic_pointer_cast<mc_tasks::RelativeEndEffectorTask>(loaded_p);
    BOOST_REQUIRE(ref);
    BOOST_REQUIRE(loaded);
    BOOST_CHECK_CLOSE(ref->positionTask->stiffness(), loaded->positionTask->stiffness(), 1e-6);
    BOOST_CHECK_CLOSE(ref->positionTask->weight(), loaded->positionTask->weight(), 1e-6);
    BOOST_CHECK_CLOSE(ref->orientationTask->stiffness(), loaded->orientationTask->stiffness(), 1e-6);
    BOOST_CHECK_CLOSE(ref->orientationTask->weight(), loaded->orientationTask->weight(), 1e-6);
    BOOST_CHECK(ref->get_ef_pose().rotation().isApprox(loaded->get_ef_pose().rotation(), 1e-6));
    BOOST_CHECK(ref->get_ef_pose().translation().isApprox(loaded->get_ef_pose().translation(), 1e-6));
  }

  double stiffness = fabs(rnd());
  double weight = fabs(rnd());
  Eigen::Matrix3d ori = Eigen::Matrix3d::Random();
  Eigen::Vector3d pos = Eigen::Vector3d::Random();
};

template<>
struct TaskTester<mc_tasks::GazeTask>
{
  mc_tasks::MetaTaskPtr make_ref()
  {
    auto ret = std::make_shared<mc_tasks::GazeTask>("NECK_P_S", Eigen::Vector3d::Zero(), X_b_gaze, *robots, 0,
                                                    stiffness, weight);
    return ret;
  }

  std::string json()
  {
    mc_rtc::Configuration config;
    config.add("type", "gaze");
    config.add("robotIndex", 0);
    config.add("body", "NECK_P_S");
    config.add("stiffness", stiffness);
    config.add("weight", weight);
    config.add("X_b_gaze", X_b_gaze);
    auto ret = getTmpFile();
    config.save(ret);
    return ret;
  }

  void check(const mc_tasks::MetaTaskPtr & ref_p, const mc_tasks::MetaTaskPtr & loaded_p)
  {
    auto ref = std::dynamic_pointer_cast<mc_tasks::GazeTask>(ref_p);
    auto loaded = std::dynamic_pointer_cast<mc_tasks::GazeTask>(loaded_p);
    BOOST_REQUIRE(ref);
    BOOST_REQUIRE(loaded);
    BOOST_CHECK_CLOSE(ref->stiffness(), loaded->stiffness(), 1e-6);
    BOOST_CHECK_CLOSE(ref->weight(), loaded->weight(), 1e-6);
  }

  double stiffness = fabs(rnd());
  double weight = fabs(rnd());
  sva::PTransformd X_b_gaze = random_pt();
};

template<>
struct TaskTester<mc_tasks::PositionBasedVisServoTask>
{
  mc_tasks::MetaTaskPtr make_ref()
  {
    auto ret = std::make_shared<mc_tasks::PositionBasedVisServoTask>("NECK_P_S", sva::PTransformd::Identity(), X_b_s,
                                                                     *robots, 0, stiffness, weight);
    return ret;
  }

  std::string json()
  {
    mc_rtc::Configuration config;
    config.add("type", "pbvs");
    config.add("robotIndex", 0);
    config.add("surface", "LeftFoot");
    config.add("stiffness", stiffness);
    config.add("weight", weight);
    auto ret = getTmpFile();
    config.save(ret);
    return ret;
  }

  void check(const mc_tasks::MetaTaskPtr & ref_p, const mc_tasks::MetaTaskPtr & loaded_p)
  {
    auto ref = std::dynamic_pointer_cast<mc_tasks::PositionBasedVisServoTask>(ref_p);
    auto loaded = std::dynamic_pointer_cast<mc_tasks::PositionBasedVisServoTask>(loaded_p);
    BOOST_REQUIRE(ref);
    BOOST_REQUIRE(loaded);
    BOOST_CHECK_CLOSE(ref->stiffness(), loaded->stiffness(), 1e-6);
    BOOST_CHECK_CLOSE(ref->weight(), loaded->weight(), 1e-6);
  }

  double stiffness = fabs(rnd());
  double weight = fabs(rnd());
  sva::PTransformd X_b_s = random_pt();
};

template<>
struct TaskTester<mc_tasks::SurfaceTransformTask>
{
  mc_tasks::MetaTaskPtr make_ref()
  {
    auto ret = std::make_shared<mc_tasks::SurfaceTransformTask>("LeftFoot", *robots, 0, stiffness, weight);
    ret->target(target);
    return ret;
  }

  std::string json()
  {
    mc_rtc::Configuration config;
    config.add("type", "surfaceTransform");
    config.add("robotIndex", 0);
    config.add("stiffness", stiffness);
    config.add("weight", weight);
    config.add("target", target);
    config.add("surface", "LeftFoot");
    auto ret = getTmpFile();
    config.save(ret);
    return ret;
  }

  void check(const mc_tasks::MetaTaskPtr & ref_p, const mc_tasks::MetaTaskPtr & loaded_p)
  {
    auto ref = std::dynamic_pointer_cast<mc_tasks::SurfaceTransformTask>(ref_p);
    auto loaded = std::dynamic_pointer_cast<mc_tasks::SurfaceTransformTask>(loaded_p);
    BOOST_REQUIRE(ref);
    BOOST_REQUIRE(loaded);
    BOOST_CHECK_CLOSE(ref->stiffness(), loaded->stiffness(), 1e-6);
    BOOST_CHECK_CLOSE(ref->weight(), loaded->weight(), 1e-6);
    BOOST_CHECK(ref->surface() == loaded->surface());
    BOOST_CHECK(ref->target().rotation().isApprox(loaded->target().rotation(), 1e-6));
    BOOST_CHECK(ref->target().translation().isApprox(loaded->target().translation(), 1e-6));
  }

  sva::PTransformd target = random_pt();
  double stiffness = fabs(rnd());
  double weight = fabs(rnd());
};

template<>
struct TaskTester<mc_tasks::VectorOrientationTask>
{
  mc_tasks::MetaTaskPtr make_ref()
  {
    auto ret = std::make_shared<mc_tasks::VectorOrientationTask>("R_WRIST_Y_S", bodyVector, targetVector, *robots, 0,
                                                                 stiffness, weight);
    return ret;
  }

  std::string json()
  {
    mc_rtc::Configuration config;
    config.add("type", "vectorOrientation");
    config.add("body", "R_WRIST_Y_S");
    config.add("bodyVector", bodyVector);
    config.add("targetVector", targetVector);
    config.add("robotIndex", 0);
    config.add("stiffness", stiffness);
    config.add("weight", weight);
    auto ret = getTmpFile();
    config.save(ret);
    return ret;
  }

  void check(const mc_tasks::MetaTaskPtr & ref_p, const mc_tasks::MetaTaskPtr & loaded_p)
  {
    auto ref = std::dynamic_pointer_cast<mc_tasks::VectorOrientationTask>(ref_p);
    auto loaded = std::dynamic_pointer_cast<mc_tasks::VectorOrientationTask>(loaded_p);
    BOOST_REQUIRE(ref);
    BOOST_REQUIRE(loaded);
    BOOST_CHECK_CLOSE(ref->stiffness(), loaded->stiffness(), 1e-6);
    BOOST_CHECK_CLOSE(ref->weight(), loaded->weight(), 1e-6);
    BOOST_CHECK(ref->body() == loaded->body());
    BOOST_CHECK(ref->bodyVector().isApprox(loaded->bodyVector()));
  }

  Eigen::Vector3d bodyVector = Eigen::Vector3d::Random();
  Eigen::Vector3d targetVector = Eigen::Vector3d::Random();
  double stiffness = fabs(rnd());
  double weight = fabs(rnd());
};

typedef boost::mpl::list<mc_tasks::CoMTask,
                         mc_tasks::AddContactTask,
                         mc_tasks::RemoveContactTask,
                         mc_tasks::force::ComplianceTask,
                         mc_tasks::OrientationTask,
                         mc_tasks::PositionTask,
                         mc_tasks::EndEffectorTask,
                         mc_tasks::RelativeEndEffectorTask,
                         mc_tasks::GazeTask,
                         mc_tasks::PositionBasedVisServoTask,
                         mc_tasks::SurfaceTransformTask,
                         mc_tasks::VectorOrientationTask>
    test_types;

BOOST_AUTO_TEST_CASE_TEMPLATE(TestMetaTaskLoader, T, test_types)
{
  auto tester = TaskTester<T>();
  auto ref = tester.make_ref();
  auto conf = tester.json();
  auto loaded = mc_tasks::MetaTaskLoader::load(solver, conf);
  tester.check(ref, loaded);
}
