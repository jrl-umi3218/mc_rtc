/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifdef BOOST_TEST_MAIN
#  undef BOOST_TEST_MAIN
#endif

#include <mc_control/fsm/Controller.h>

#include <mc_control/mc_controller.h>

#include <mc_rtc/logging.h>

#include <mc_tasks/CoMTask.h>

#include <boost/test/unit_test.hpp>

namespace mc_control
{

struct MC_CONTROL_DLLAPI TestFSMStateOptionsController : public fsm::Controller
{
public:
  TestFSMStateOptionsController(mc_rbdyn::RobotModulePtr rm,
                                double dt,
                                const mc_rtc::Configuration & conf,
                                Backend backend)
  : fsm::Controller(rm, dt, conf, backend)
  {
    // Check that the default constructor loads the robot + ground environment
    BOOST_CHECK_EQUAL(robots().size(), 2);
    // Check that JVRC-1 was loaded
    BOOST_CHECK_EQUAL(robot().name(), "jvrc1");
    BOOST_REQUIRE(hasRobot("ground"));
    mc_rtc::log::success("Created TestFSMStateOptionsController");
  }

  bool run() override
  {
    bool ret = fsm::Controller::run();
    BOOST_REQUIRE(ret);
    Eigen::Vector6d dof = Eigen::Vector6d::Ones();
    dof(2) = 0.0;
    if(iter_ == 0) // TestContactsManipulation
    {
      BOOST_REQUIRE(executor_.state() == "TestContactsManipulation");
      // The AddContacts/RemoveContacts actions should have swapped the contacts
      BOOST_REQUIRE(!hasContact("LeftFoot"));
      BOOST_REQUIRE(!hasContact("RightFoot"));
      BOOST_REQUIRE(hasContact("LeftFootCenter", 0.8));
      BOOST_REQUIRE(hasContact("RightFootCenter", 0.8, dof));
      // note: contacts are added to the QP at the start of the next call to mc_controller::run.
      // Thus this checks for the contacts added on the previous iteration
      BOOST_REQUIRE(contacts().size() == 2);
    }
    if(iter_ == 1) // TestContactsManipulation2
    {
      BOOST_REQUIRE(executor_.state() == "TestContactsManipulation2");
      BOOST_REQUIRE(solver().contacts().size() == 2);
      for(const auto & c : solver().contacts())
      {
        if(c.r1Surface()->name() == "RightFootCenter") { BOOST_REQUIRE(c.dof() == dof); }
      }
    }
    else if(iter_ == 2)
    {
      BOOST_REQUIRE(solver().contacts().size() == 2);
      BOOST_REQUIRE(executor_.state() == "TestCollisionsManipulation");
      BOOST_REQUIRE(hasCollision("jvrc1", "ground", {"L_WRIST_Y_S", "ground", 0.05, 0.01, 0}));
      BOOST_REQUIRE(!hasCollision("jvrc1", "jvrc1", {"R_WRIST_Y_S", "R_HIP_Y_S", 0.05, 0.025, 0}));
    }
    else
    {
      BOOST_REQUIRE(!hasCollision("jvrc1", "ground", {"L_WRIST_Y_S", "ground", 0.05, 0.01, 0}));
      BOOST_REQUIRE(hasCollision("jvrc1", "jvrc1", {"R_WRIST_Y_S", "R_HIP_Y_S", 0.05, 0.025, 0}));
    }
    if(iter_ == 3) // TestRemovePostureTask
    {
      BOOST_REQUIRE(executor_.state() == "TestRemovePostureTask");
      // AddContactsAfter/RemoveContactsAfter should have put the contacts back
      BOOST_REQUIRE(hasContact("LeftFoot"));
      BOOST_REQUIRE(hasContact("RightFoot"));
      BOOST_REQUIRE(!hasContact("LeftFootCenter"));
      BOOST_REQUIRE(!hasContact("RightFootCenter"));
      // RemovePostureTask should have removed the posture task
      BOOST_REQUIRE(!getPostureTask("jvrc1")->inSolver());
    }
    else { BOOST_REQUIRE(getPostureTask("jvrc1")->inSolver()); }
    if(iter_ > 3) // TestConstraintsAndTasks
    {
      BOOST_REQUIRE(executor_.state() == "TestConstraintsAndTasks");
      // There is now a constraint to set l_wrist speed to a constant
      auto bIndex = robot().bodyIndexByName("l_wrist");
      auto speed = robot().bodyVelB()[bIndex].vector();
      Eigen::Vector6d ref = Eigen::Vector6d::Zero();
      ref(5) = 0.01;
      BOOST_REQUIRE((speed - ref).norm() < 5e-4);
      BOOST_REQUIRE(hasTask<mc_tasks::CoMTask>());
    }
    else { BOOST_REQUIRE(!hasTask<mc_tasks::CoMTask>()); }
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
    fsm::Contact empty_r1_robot_c("", "ground", s, "AllGround", friction, dof);
    if(hasContact(empty_r1_robot_c))
    {
      mc_rtc::log::error("Found a contact with an empty r1 robot name, this shouldn't be possible {}", s);
      // There used to be a bug where inserting a contact with an empty r1/r2 robot
      // would add a contact with an empty robot name, thus leading to duplicated
      // contacts when one would add the same contact by explicitely specifing r1/r2
      return false;
    }
    fsm::Contact empty_r2_robot_c("jvrc1", "", s, "AllGround", friction, dof);
    if(hasContact(empty_r2_robot_c))
    {
      mc_rtc::log::error("Found a contact with an empty r2 robot name, this shouldn't be possible {}", s);
      return false;
    }
    const auto & ref = contact(c);
    return ref.friction == c.friction && ref.dof == c.dof;
  }

  bool hasCollision(const std::string & r1, const std::string & r2, const mc_rbdyn::Collision & col)
  {
    if(!collision_constraints_.count({r1, r2})) { return false; }
    const auto & cols = collision_constraints_.at({r1, r2})->cols;
    return std::find(cols.begin(), cols.end(), col) != cols.end();
  }

  template<typename T>
  bool hasTask()
  {
    for(const auto & t : solver().tasks())
    {
      if(dynamic_cast<const T *>(t) != nullptr) { return true; }
    }
    return false;
  }

private:
  unsigned int iter_ = 0;
};

} // namespace mc_control

using Controller = mc_control::TestFSMStateOptionsController;
using Backend = mc_control::MCController::Backend;
MULTI_CONTROLLERS_CONSTRUCTOR("TestFSMStateOptions",
                              Controller(rm, dt, config, Backend::Tasks),
                              "TestFSMStateOptions_TVM",
                              Controller(rm, dt, config, Backend::TVM))
