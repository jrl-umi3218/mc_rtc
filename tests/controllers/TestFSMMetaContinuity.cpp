/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifdef BOOST_TEST_MAIN
#  undef BOOST_TEST_MAIN
#endif

#include <mc_control/fsm/Controller.h>
#include <mc_control/mc_controller.h>
#include <boost/test/unit_test.hpp>

namespace mc_control
{

struct MC_CONTROL_DLLAPI TestFSMMetaContinuityController : public fsm::Controller
{
public:
  TestFSMMetaContinuityController(mc_rbdyn::RobotModulePtr rm,
                                  double dt,
                                  const mc_rtc::Configuration & conf,
                                  Backend backend)
  : fsm::Controller(rm, dt, conf, backend)
  {
    datastore().make<unsigned>("ControllerIter", 0u);
    datastore().make<unsigned>("StateIter", 0u);
    mc_rtc::log::success("Created TestFSMMetaContinuityController");
  }

  bool run() override
  {
    bool ret = fsm::Controller::run();
    BOOST_REQUIRE(ret);

    auto & controllerIter = datastore().get<unsigned>("ControllerIter");
    auto & stateIter = datastore().get<unsigned>("StateIter");
    BOOST_REQUIRE_EQUAL(++controllerIter, stateIter);
    return ret;
  }
};

} // namespace mc_control

using Controller = mc_control::TestFSMMetaContinuityController;
using Backend = mc_control::MCController::Backend;
MULTI_CONTROLLERS_CONSTRUCTOR("TestFSMMetaContinuity",
                              Controller(rm, dt, config, Backend::Tasks),
                              "TestFSMMetaContinuity_TVM",
                              Controller(rm, dt, config, Backend::TVM))
