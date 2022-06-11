/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifdef BOOST_TEST_MAIN
#  undef BOOST_TEST_MAIN
#endif

#include <mc_control/api.h>
#include <mc_control/mc_controller.h>

#include <mc_tasks/MomentumTask.h>

#include <mc_rtc/logging.h>

#include <boost/test/unit_test.hpp>

namespace mc_control
{

struct MC_CONTROL_DLLAPI TestMomentumTaskController : public MCController
{
public:
  TestMomentumTaskController(mc_rbdyn::RobotModulePtr rm, double dt) : MCController(rm, dt)
  {
    // Check that the default constructor loads the robot + ground environment
    BOOST_CHECK_EQUAL(robots().size(), 2);
    // Check that JVRC-1 was loaded
    BOOST_CHECK_EQUAL(robot().name(), "jvrc1");
    solver().addConstraintSet(contactConstraint);
    solver().addConstraintSet(dynamicsConstraint);
    postureTask->stiffness(1);
    postureTask->weight(1);
    solver().addTask(postureTask.get());
    addContact({robot().name(), "ground", "LeftFoot", "AllGround"});
    addContact({robot().name(), "ground", "RightFoot", "AllGround"});

    /* Create and add the momentum task with the default stiffness/weight */
    momentumTask = std::make_shared<mc_tasks::MomentumTask>(robots(), 0);
    momentumTask->stiffness(10);
    solver().addTask(momentumTask);

    mc_rtc::log::success("Created TestMomentumTaskController");
  }

  virtual bool run() override
  {
    bool ret = MCController::run();
    if(!ret)
    {
      mc_rtc::log::critical("Failed at iter: {}", nrIter);
    }
    BOOST_CHECK(ret);
    nrIter++;
    if(nrIter == 500)
    {
      // Swap the contact order
      removeContact({robot().name(), "ground", "LeftFoot", "AllGround"});
      removeContact({robot().name(), "ground", "RightFoot", "AllGround"});
      addContact({"ground", robot().name(), "AllGround", "LeftFoot"});
      addContact({"ground", robot().name(), "AllGround", "RightFoot"});
      momentumTask->momentum(sva::ForceVecd::Zero());
    }
    if(nrIter == 1000)
    {
      /* Check that the task is "finished" */
      BOOST_CHECK_SMALL(momentumTask->eval().norm(), 1e-2);
      BOOST_CHECK_SMALL(momentumTask->speed().norm(), 1e-3);

      /* Apply dimWeight and give an unreachable momentum target */
      Eigen::Vector6d dimW = Eigen::Vector6d::Ones();
      dimW(5) = 0.0;
      momentumTask->dimWeight(dimW);
      momentumTask->momentum({Eigen::Vector3d::Zero(), Eigen::Vector3d(0., 0., 100.)});
      postureTask->posture(robot().mbc().q);
    }
    if(nrIter == 2000)
    {
      BOOST_CHECK_SMALL(momentumTask->eval().norm(), 1e-2);
      BOOST_CHECK_SMALL(momentumTask->speed().norm(), 1e-3);
    }
    return ret;
  }

  virtual void reset(const ControllerResetData & reset_data) override
  {
    MCController::reset(reset_data);
    momentumTask->reset();
    /** Raise the momentum */
    momentumTask->momentum({Eigen::Vector3d::Zero(), Eigen::Vector3d(0., 0., 1.)});
  }

private:
  unsigned int nrIter = 0;
  std::shared_ptr<mc_tasks::MomentumTask> momentumTask = nullptr;
};

} // namespace mc_control

using Controller = mc_control::TestMomentumTaskController;
SIMPLE_CONTROLLER_CONSTRUCTOR("TestMomentumTaskController", Controller)
