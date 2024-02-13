/*
 * Copyright 2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifdef BOOST_TEST_MAIN
#  undef BOOST_TEST_MAIN
#endif
#include <mc_control/api.h>
#include <mc_control/mc_controller.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/PostureTask.h>

#include <boost/test/unit_test.hpp>

namespace mc_control
{

struct MC_CONTROL_DLLAPI TestCanonicalRobotController : public MCController
{
public:
  TestCanonicalRobotController(std::shared_ptr<mc_rbdyn::RobotModule> rm, double dt, Backend backend)
  : MCController(rm, dt, backend)
  {
    // Check that the default constructor loads the robot + ground environment
    BOOST_CHECK_EQUAL(robots().size(), 2);
    // Check that JVRC-1 was loaded
    BOOST_CHECK_EQUAL(robot().name(), "jvrc1");
    BOOST_CHECK_EQUAL(rm->_parameters[0], "JVRC1NoHands");
    BOOST_CHECK_EQUAL(rm->_canonicalParameters[0], "JVRC1");
    // Check that RobotData is shared between all robots
    for(const auto & r : robots())
    {
      BOOST_REQUIRE(r.data().get() == realRobot(r.name()).data().get());
      BOOST_REQUIRE(r.data().get() == outputRobot(r.name()).data().get());
      BOOST_REQUIRE(r.data().get() == outputRealRobot(r.name()).data().get());
    }
    auto for_all_robots = [this](auto && callback)
    {
      callback(robot());
      callback(realRobot());
      callback(outputRobot());
      callback(outputRealRobot());
    };
    auto require_for_all_robots = [this](auto && callback, const char * desc)
    {
      auto do_throw = [&]()
      {
        mc_rtc::log::error_and_throw(desc);
        return true;
      };
      BOOST_REQUIRE(callback(robot()) || do_throw());
      BOOST_REQUIRE(callback(realRobot()) || do_throw());
      BOOST_REQUIRE(callback(outputRobot()) || do_throw());
      BOOST_REQUIRE(callback(outputRealRobot()) || do_throw());
    };
    // Check that we can create frames at runtime
    require_for_all_robots([](const mc_rbdyn::Robot & r) { return !r.hasFrame("CameraFrame"); }, "has no CameraFrame");
    for_all_robots([](mc_rbdyn::Robot & r)
                   { r.makeFrame("CameraFrame", r.frame("NECK_P_S"), sva::PTransformd::Identity()); });
    require_for_all_robots([](const mc_rbdyn::Robot & r) { return r.hasFrame("CameraFrame"); }, "has CameraFrame");
    // Checks that we can add a new force sensor at run-time
    require_for_all_robots([](const mc_rbdyn::Robot & r) { return !r.hasForceSensor("HeadForceSensor"); },
                           "has no HeadForceSensor");
    require_for_all_robots([](const mc_rbdyn::Robot & r) { return !r.bodyHasForceSensor("NECK_P_S"); },
                           "NECK_P_S has no force sensor");
    require_for_all_robots([](const mc_rbdyn::Robot & r) { return !r.frame("CameraFrame").hasForceSensor(); },
                           "CameraFrame has no force sensor");
    robot().addForceSensor(mc_rbdyn::ForceSensor{"HeadForceSensor", "NECK_P_S", sva::PTransformd::Identity()});
    require_for_all_robots([](const mc_rbdyn::Robot & r) { return r.hasForceSensor("HeadForceSensor"); },
                           "has HeadForceSensor");
    require_for_all_robots([](const mc_rbdyn::Robot & r) { return r.bodyHasForceSensor("NECK_P_S"); },
                           "NECK_P_S has force sensor");
    require_for_all_robots([](const mc_rbdyn::Robot & r) { return r.frame("CameraFrame").hasForceSensor(); },
                           "CameraFrame has force sensor");
    solver().addConstraintSet(kinematicsConstraint);
    postureTask->stiffness(1);
    postureTask->weight(1);
    solver().addTask(postureTask.get());
    mc_rtc::log::success("Created TestCanonicalRobotController");
  }

  virtual bool run() override
  {
    BOOST_REQUIRE(datastore().get<bool>("HasTestCanonicalRobotControllerPlugin"));
    auto njoints = robot().mbc().q.size();
    auto postureT = [&](size_t j)
    { return std::cos(static_cast<double>(j) / static_cast<double>(njoints) + nrIter / 2000.); };

    size_t jIdx = 0;
    for(const auto & j : robot().mb().joints())
    {
      if(j.dof() == 1) { target_[j.name()] = {postureT(jIdx)}; }
      jIdx++;
    }
    postureTask->target(target_);

    double & targetQ = datastore().get<double>("GripperTarget");
    if(targetQ >= gripper_max_ && dir_ > 0.0) { dir_ = -1.0; }
    if(targetQ <= gripper_min_ && dir_ < 0.0) { dir_ = 1.0; }
    targetQ += dir_ * vmax_ * solver().dt();

    robot().gripper("l_gripper").setTargetQ("L_UTHUMB", targetQ);
    robot().gripper("r_gripper").setTargetQ("R_UTHUMB", targetQ);

    bool ret = MCController::run();
    BOOST_CHECK(ret);
    nrIter++;
    return ret;
  }

  virtual void reset(const ControllerResetData & reset_data) override
  {
    MCController::reset(reset_data);
    datastore().make<double>("GripperTarget", robot().gripper("l_gripper").getTargetQ("L_UTHUMB"));
    // Use half of the configured max speed of the gripper
    gripper_min_ = outputRobot().module().bounds()[0].at("L_UTHUMB")[0];
    gripper_max_ = outputRobot().module().bounds()[1].at("L_UTHUMB")[0];
    double gripper_range = (gripper_max_ - gripper_min_);
    gripper_min_ += 0.1 * gripper_range;
    gripper_max_ -= 0.1 * gripper_range;
    vmax_ = outputRobot().module().bounds()[3].at("L_UTHUMB")[0] * robot().gripper("l_gripper").percentVMAX() * 0.5;
  }

private:
  unsigned int nrIter = 0;
  std::map<std::string, std::vector<double>> target_;
  double gripper_min_ = 0.0;
  double gripper_max_ = 0.0;
  double vmax_ = 0.0;
  double dir_ = 1.0;
};

} // namespace mc_control

using Controller = mc_control::TestCanonicalRobotController;
using Backend = mc_control::MCController::Backend;
MULTI_CONTROLLERS_CONSTRUCTOR("TestCanonicalRobotController",
                              Controller(rm, dt, Backend::Tasks),
                              "TestCanonicalRobotController_TVM",
                              Controller(rm, dt, Backend::TVM))
