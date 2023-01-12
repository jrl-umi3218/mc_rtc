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
  TestRobotConfigurationControllerController(mc_rbdyn::RobotModulePtr rm, double dt, Backend backend)
  : MCController(rm, dt, backend)
  {
    // Check that the default constructor loads the robot + ground environment
    BOOST_CHECK_EQUAL(robots().size(), 2);
    // Check that JVRC-1 was loaded
    BOOST_CHECK_EQUAL(robot().name(), "jvrc1");
    qpsolver->addConstraintSet(contactConstraint);
    qpsolver->addConstraintSet(kinematicsConstraint);
    qpsolver->addConstraintSet(selfCollisionConstraint);
    qpsolver->addConstraintSet(*compoundJointConstraint);
    qpsolver->addTask(postureTask.get());
    qpsolver->setContacts({});
    check_robot_config();
    mc_rtc::log::success("Created TestRobotConfigurationControllerController");
  }

  virtual bool run() override
  {
    return MCController::run();
  }

  virtual void reset(const ControllerResetData & reset_data) override
  {
    check_robot_config();
    MCController::reset(reset_data);
  }

  void check_robot_config()
  {
    auto config = robot_config(robot());
    BOOST_REQUIRE(config.has("MagicSetting"));
    double setting = config("MagicSetting");
    BOOST_REQUIRE(std::fabs(setting - 42.42) < 1e-6);
  }

private:
};

} // namespace mc_control

using Controller = mc_control::TestRobotConfigurationControllerController;
using Backend = mc_control::MCController::Backend;
MULTI_CONTROLLERS_CONSTRUCTOR("TestRobotConfigurationController",
                              Controller(rm, dt, Backend::Tasks),
                              "TestRobotConfigurationController_TVM",
                              Controller(rm, dt, Backend::TVM))
