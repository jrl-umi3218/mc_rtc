/*
 * Copyright 2023 CNRS-UM LIRMM, CNRS-AIST JRL
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

struct MC_CONTROL_DLLAPI TestObserverConfigurationControllerController : public MCController
{
public:
  TestObserverConfigurationControllerController(mc_rbdyn::RobotModulePtr rm,
                                                double dt,
                                                const mc_rtc::Configuration & config,
                                                Backend backend)
  : MCController(rm, dt, config, backend)
  {
    // Check that the default constructor loads the robot + ground environment
    BOOST_REQUIRE_EQUAL(robots().size(), 2);
    // Check that JVRC-1 was loaded
    BOOST_REQUIRE_EQUAL(robot().name(), "jvrc1");
    qpsolver->addConstraintSet(contactConstraint);
    qpsolver->addConstraintSet(kinematicsConstraint);
    qpsolver->addConstraintSet(selfCollisionConstraint);
    qpsolver->addConstraintSet(*compoundJointConstraint);
    qpsolver->addTask(postureTask.get());
    mc_rtc::log::success("Created TestObserverConfigurationControllerController");
  }

  virtual bool run() override { return MCController::run(); }

  virtual void reset(const ControllerResetData & reset_data) override { MCController::reset(reset_data); }

private:
};

} // namespace mc_control

using Controller = mc_control::TestObserverConfigurationControllerController;
using Backend = mc_control::MCController::Backend;
MULTI_CONTROLLERS_CONSTRUCTOR("TestObserverConfigurationController",
                              Controller(rm, dt, config, Backend::Tasks),
                              "TestObserverConfigurationController_TVM",
                              Controller(rm, dt, config, Backend::TVM))
