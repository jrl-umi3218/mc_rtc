/*
 * Copyright 2015-2023 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifdef BOOST_TEST_MAIN
#  undef BOOST_TEST_MAIN
#endif

#include <mc_control/fsm/Controller.h>

#include <mc_control/mc_controller.h>

#include <mc_rtc/logging.h>

#include <boost/test/unit_test.hpp>

namespace mc_control
{

struct MC_CONTROL_DLLAPI TestPythonStateController : public fsm::Controller
{
public:
  TestPythonStateController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & conf, Backend backend)
  : fsm::Controller(rm, dt, conf, backend)
  {
    // Check that the default constructor loads the robot + ground environment
    BOOST_CHECK_EQUAL(robots().size(), 2);
    // Check that JVRC-1 was loaded
    BOOST_CHECK_EQUAL(robot().name(), "jvrc1");
    BOOST_REQUIRE(hasRobot("ground"));
    mc_rtc::log::success("Created TestPythonStateController");
  }

  bool run() override
  {
    bool ret = fsm::Controller::run();
    BOOST_REQUIRE(ret);
    if(iter_ == 0) { BOOST_REQUIRE(executor_.state() == "TestInitial"); }
    else if(iter_ == 1)
    {
      BOOST_REQUIRE(executor_.next_state() == "TestFinal");
      BOOST_REQUIRE(executor_.output() == "Value=42");
      executor_.next();
    }
    else
    {
      BOOST_REQUIRE(executor_.state() == "TestFinal");
      if(iter_ > 2)
      {
        BOOST_REQUIRE(executor_.complete());
        BOOST_REQUIRE(executor_.output() == "Value=100");
      }
    }
    iter_++;
    return ret;
  }

  void reset(const ControllerResetData & reset_data) override
  {
    fsm::Controller::reset(reset_data);
    BOOST_REQUIRE(hasContact("LeftFoot"));
    BOOST_REQUIRE(hasContact("RightFoot"));
  }

  using fsm::Controller::hasContact;

  const fsm::Contact & contact(const fsm::Contact & c)
  {
    assert(hasContact(c));
    return *contacts().find(c);
  }

  bool hasContact(const std::string & s,
                  double friction = mc_rbdyn::Contact::defaultFriction,
                  const Eigen::Vector6d & dof = Eigen::Vector6d::Ones())
  {
    fsm::Contact c("jvrc1", "ground", s, "AllGround", friction, dof);
    if(!hasContact(c)) { return false; }
    const auto & ref = contact(c);
    return ref.friction == c.friction && ref.dof == c.dof;
  }

private:
  unsigned int iter_ = 0;
};

} // namespace mc_control

using Controller = mc_control::TestPythonStateController;
using Backend = mc_control::MCController::Backend;
MULTI_CONTROLLERS_CONSTRUCTOR("TestPythonState",
                              Controller(rm, dt, config, Backend::Tasks),
                              "TestPythonState_TVM",
                              Controller(rm, dt, config, Backend::TVM))
