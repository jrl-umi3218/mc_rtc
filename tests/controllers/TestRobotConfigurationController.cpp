/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifdef BOOST_TEST_MAIN
#  undef BOOST_TEST_MAIN
#endif
#include <mc_control/api.h>
#include <mc_control/mc_controller.h>
#include <mc_rtc/logging.h>

#include <boost/test/unit_test.hpp>

namespace mc_control
{

struct MC_CONTROL_DLLAPI TestRobotConfigurationControllerController : public MCController
{
public:
  TestRobotConfigurationControllerController(mc_rbdyn::RobotModulePtr rm,
                                             double dt,
                                             const mc_rtc::Configuration & config,
                                             Backend backend)
  : MCController(rm, dt, config, ControllerParameters{backend}.load_robot_config_with_module_name(false))
  {
    // Check that the default constructor loads the robot + ground environment
    BOOST_REQUIRE_EQUAL(robots().size(), 3);
    // Check that JVRC-1 was loaded
    BOOST_REQUIRE_EQUAL(robot().name(), "jvrc1");
    BOOST_REQUIRE(hasRobot("jvrc2"));
    qpsolver->addConstraintSet(contactConstraint);
    qpsolver->addConstraintSet(kinematicsConstraint);
    qpsolver->addConstraintSet(selfCollisionConstraint);
    qpsolver->addConstraintSet(*compoundJointConstraint);
    qpsolver->addTask(postureTask.get());
    qpsolver->setContacts({});
    check_controller_config();
    check_robots_config();
    mc_rtc::log::success("Created TestRobotConfigurationControllerController");
  }

  virtual bool run() override
  {
    return MCController::run();
  }

  virtual void reset(const ControllerResetData & reset_data) override
  {
    check_robots_config();
    check_controller_config();
    MCController::reset(reset_data);
  }

  void check_config(const mc_rtc::Configuration & config, double expected)
  {
    BOOST_REQUIRE(config.has("MagicSetting"));
    double setting = config("MagicSetting");
    BOOST_REQUIRE(std::fabs(setting - expected) < 1e-6);
  }

  void check_robots_config()
  {
    check_config(robot_config(robot()), 42.42);
    check_config(robot_config("jvrc2"), 100.0);
  }

  void check_controller_config()
  {
    BOOST_REQUIRE(config_.has("robots"));
    auto c_robots = config_("robots");
    BOOST_REQUIRE(c_robots.has("jvrc1"));
    auto c_jvrc1 = c_robots("jvrc1");
    check_config(c_jvrc1, 42.42);
    BOOST_REQUIRE(c_robots.has("jvrc2"));
    auto c_jvrc2 = c_robots("jvrc2");
    check_config(c_jvrc2, 100.0);
  }

private:
};

} // namespace mc_control

using Controller = mc_control::TestRobotConfigurationControllerController;
using Backend = mc_control::MCController::Backend;
MULTI_CONTROLLERS_CONSTRUCTOR("TestRobotConfigurationController",
                              Controller(rm, dt, config, Backend::Tasks),
                              "TestRobotConfigurationController_TVM",
                              Controller(rm, dt, config, Backend::TVM))
