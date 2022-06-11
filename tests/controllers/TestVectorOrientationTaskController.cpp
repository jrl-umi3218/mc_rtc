/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifdef BOOST_TEST_MAIN
#  undef BOOST_TEST_MAIN
#endif

#include <mc_control/api.h>
#include <mc_control/mc_controller.h>

#include <mc_tasks/VectorOrientationTask.h>

#include <mc_rtc/logging.h>

#include <boost/test/unit_test.hpp>

namespace mc_control
{

struct MC_CONTROL_DLLAPI TestVectorOrientationTaskController : public MCController
{
public:
  TestVectorOrientationTaskController(mc_rbdyn::RobotModulePtr rm, double dt) : MCController(rm, dt)
  {
    // Check that the default constructor loads the robot + ground environment
    BOOST_CHECK_EQUAL(robots().size(), 2);
    // Check that JVRC-1 was loaded
    BOOST_CHECK_EQUAL(robot().name(), "jvrc1");
    solver().addConstraintSet(contactConstraint);
    solver().addConstraintSet(kinematicsConstraint);
    postureTask->stiffness(2);
    postureTask->weight(100);
    solver().addTask(postureTask.get());
    addContact({robot().name(), "ground", "LeftFoot", "AllGround"});
    addContact({robot().name(), "ground", "RightFoot", "AllGround"});

    /* Create and add the momentum task with the default stiffness/weight */
    voTask = std::make_shared<mc_tasks::VectorOrientationTask>(robot().frame("NECK_P_S"), Eigen::Vector3d::UnitX());
    voTask->stiffness(10);
    solver().addTask(voTask);

    mc_rtc::log::success("Created TestVectorOrientationTaskController");
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
      voTask->stiffness(100);
    }
    if(nrIter == 1000)
    {
      /* Check that the task is "finished" */
      BOOST_CHECK_SMALL(voTask->eval().norm(), 1e-2);
      BOOST_CHECK_SMALL(voTask->speed().norm(), 1e-3);
      solver().removeTask(voTask);
      voTask =
          std::make_shared<mc_tasks::VectorOrientationTask>(robot().frame("RightGripper"), Eigen::Vector3d::UnitZ());
      voTask->targetVector(Eigen::Vector3d::UnitX());
      voTask->stiffness(10.0);
      solver().addTask(voTask);
    }
    if(nrIter == 1500)
    {
      voTask->stiffness(100);
    }
    if(nrIter == 2000)
    {
      BOOST_CHECK_SMALL(voTask->eval().norm(), 0.02);
      BOOST_CHECK_SMALL(voTask->speed().norm(), 1e-3);
    }
    return ret;
  }

  virtual void reset(const ControllerResetData & reset_data) override
  {
    MCController::reset(reset_data);
    voTask->reset();
    voTask->targetVector(sva::RotZ(0.75) * Eigen::Vector3d::UnitX());
  }

private:
  unsigned int nrIter = 0;
  std::shared_ptr<mc_tasks::VectorOrientationTask> voTask = nullptr;
};

} // namespace mc_control

using Controller = mc_control::TestVectorOrientationTaskController;
SIMPLE_CONTROLLER_CONSTRUCTOR("TestVectorOrientationTaskController", Controller)
