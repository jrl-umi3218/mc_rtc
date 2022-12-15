/*
 * Copyright 2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/GlobalPluginMacros.h>

#ifdef BOOST_TEST_MAIN
#  undef BOOST_TEST_MAIN
#endif
#include <boost/test/unit_test.hpp>

namespace mc_plugin
{

struct MC_CONTROL_DLLAPI TestCanonicalRobotController : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & gc, const mc_rtc::Configuration &) override
  {
    reset(gc);
  }

  void reset(mc_control::MCGlobalController & gc) override
  {
    gc.controller().datastore().make<bool>("HasTestCanonicalRobotControllerPlugin", true);
  }

  void before(mc_control::MCGlobalController &) override {}

  void after(mc_control::MCGlobalController & controller) override
  {
    const auto & robot = controller.controller().robot();
    const auto & outputRobot = controller.controller().outputRobot();
    // For all non gripper joints
    for(const auto & j : robot.refJointOrder())
    {
      if(robot.hasJoint(j))
      {
        size_t jIdx = robot.jointIndexByName(j);
        size_t oIdx = outputRobot.jointIndexByName(j);
        if(!robot.mb().joint(static_cast<int>(jIdx)).isMimic())
        {
          BOOST_REQUIRE_CLOSE(robot.mbc().q[jIdx][0], outputRobot.mbc().q[oIdx][0], 1e-10);
          BOOST_REQUIRE_CLOSE(robot.mbc().alpha[jIdx][0], outputRobot.mbc().alpha[oIdx][0], 1e-10);
          BOOST_REQUIRE_CLOSE(robot.mbc().alphaD[jIdx][0], outputRobot.mbc().alphaD[oIdx][0], 1e-10);
        }
      }
    }

    // For all gripper joints
    auto commanded = controller.controller().datastore().get<double>("GripperTarget");
    for(const auto & gripper : robot.grippers())
    {
      // Get the commanded angle that the mc_control::Gripper should have sent
      // to the output robot
      for(const auto & j : gripper.get().activeJoints())
      {
        BOOST_REQUIRE(std::fabs(commanded - outputRobot.mbc().q[outputRobot.jointIndexByName(j)][0]) < 1e-10);
      }

      // Check mimics
      for(const auto & jname : gripper.get().joints())
      {
        auto jIdx = outputRobot.jointIndexByName(jname);
        const auto & j = outputRobot.mb().joint(static_cast<int>(jIdx));
        if(j.isMimic())
        {
          auto mainIndex = outputRobot.jointIndexByName(j.mimicName());
          auto mimicQ = j.mimicMultiplier() * outputRobot.mbc().q[mainIndex][0] + j.mimicOffset();
          BOOST_REQUIRE(outputRobot.mbc().q[jIdx][0] == mimicQ);
        }
      }
    }
  }
};

} // namespace mc_plugin

extern "C"
{

  GLOBAL_PLUGIN_API void MC_RTC_GLOBAL_PLUGIN(std::vector<std::string> & names)
  {
    names = {"TestCanonicalRobotController"};
  }

  GLOBAL_PLUGIN_API void destroy(mc_control::GlobalPlugin * ptr)
  {
    delete ptr;
  }

  GLOBAL_PLUGIN_API mc_control::GlobalPlugin * create(const std::string &)
  {
    return new mc_plugin::TestCanonicalRobotController();
  }
}
