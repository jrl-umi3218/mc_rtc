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
  TestCanonicalRobotController(std::shared_ptr<mc_rbdyn::RobotModule> rm, double dt) : MCController(rm, dt)
  {
    // Check that the default constructor loads the robot + ground environment
    BOOST_CHECK_EQUAL(robots().size(), 2);
    // Check that JVRC-1 was loaded
    BOOST_CHECK_EQUAL(robot().name(), "jvrc1");
    BOOST_CHECK_EQUAL(rm->_parameters[0], "JVRC1NoHands");
    BOOST_CHECK_EQUAL(rm->_canonicalParameters[0], "JVRC1");
    solver().addConstraintSet(kinematicsConstraint);
    postureTask->stiffness(1);
    postureTask->weight(1);
    solver().addTask(postureTask.get());
    mc_rtc::log::success("Created TestCanonicalRobotController");
  }

  virtual bool run() override
  {
    // FIXME
    // The outputRobot update is handled by MCGlobalController, thus we have to wait one
    // iteration to have it up to date in the controller. This is done this way
    // because the gripper control is currently handled by MCGlobalController
    // AFTER MCController::run.
    //
    // The gripper state should be handled by MCController::run instead of
    // MCGlobalController::run and the update of outputRobot handled by
    // MCController instead
    if(nrIter > 0)
    {
      // For all non gripper joints
      for(const auto & j : robot().refJointOrder())
      {
        if(j != "L_UTHUMB" && j != "R_UTHUMB")
        {
          if(!robot().mb().joint(robot().jointIndexByName(j)).isMimic())
          {
            BOOST_REQUIRE_CLOSE(robot().mbc().q[robot().jointIndexByName(j)][0],
                                outputRobot().mbc().q[outputRobot().jointIndexByName(j)][0], 1e-10);
            BOOST_REQUIRE_CLOSE(robot().mbc().alpha[robot().jointIndexByName(j)][0],
                                outputRobot().mbc().alpha[outputRobot().jointIndexByName(j)][0], 1e-10);
            BOOST_REQUIRE_CLOSE(robot().mbc().alphaD[robot().jointIndexByName(j)][0],
                                outputRobot().mbc().alphaD[outputRobot().jointIndexByName(j)][0], 1e-10);
          }
        }
      }

      // For all gripper joints
      for(const auto & gripper : robot().grippers())
      {
        // Get the commanded angle that the mc_control::Gripper should have sent
        // to the output robot
        const auto & gq = gripper.get().getGripperCommand();
        for(const auto & j : gripper.get().activeJoints())
        {
          auto commanded = gq.at(j)[0];
          BOOST_REQUIRE_CLOSE(commanded, outputRobot().mbc().q[outputRobot().jointIndexByName(j)][0], 1e-10);
        }

        // Check mimics
        for(const auto & jname : gripper.get().joints())
        {
          auto jIdx = outputRobot().jointIndexByName(jname);
          const auto & j = outputRobot().mb().joint(jIdx);
          if(j.isMimic())
          {
            auto mainIndex = outputRobot().jointIndexByName(j.mimicName());
            auto mimicQ = j.mimicMultiplier() * outputRobot().mbc().q[mainIndex][0] + j.mimicOffset();
            BOOST_REQUIRE(outputRobot().mbc().q[jIdx][0] == mimicQ);
          }
        }
      }
    }

    auto njoints = robot().mbc().q.size();
    auto postureT = [&](int j) {
      return std::cos(static_cast<double>(j) / static_cast<double>(njoints) + nrIter / 2000.);
    };

    auto posture = robot().mbc().q;
    for(unsigned i = 0; i < robot().refJointOrder().size(); ++i)
    {
      posture[robot().jointIndexInMBC(i)][0] = postureT(i);
    }
    postureTask->posture(posture);

    robot().gripper("l_gripper").setTargetQ("L_UTHUMB", postureT(0));
    robot().gripper("r_gripper").setTargetQ("R_UTHUMB", postureT(0));

    bool ret = MCController::run();
    BOOST_CHECK(ret);
    nrIter++;
    return ret;
  }

  virtual void reset(const ControllerResetData & reset_data) override
  {
    MCController::reset(reset_data);
  }

private:
  unsigned int nrIter = 0;
};

} // namespace mc_control

SIMPLE_CONTROLLER_CONSTRUCTOR("TestCanonicalRobotController", mc_control::TestCanonicalRobotController)
