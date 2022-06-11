/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifdef BOOST_TEST_MAIN
#  undef BOOST_TEST_MAIN
#endif

#include <mc_control/api.h>
#include <mc_control/mc_controller.h>

#include <mc_tasks/PositionBasedVisServoTask.h>

#include <mc_rtc/logging.h>

#include <boost/test/unit_test.hpp>

namespace mc_control
{

struct MC_CONTROL_DLLAPI TestPBVSTaskController : public MCController
{
public:
  TestPBVSTaskController(mc_rbdyn::RobotModulePtr rm, double dt) : MCController(rm, dt)
  {
    // Check that the default constructor loads the robot + ground environment
    BOOST_CHECK_EQUAL(robots().size(), 2);
    // Check that JVRC-1 was loaded
    BOOST_CHECK_EQUAL(robot().name(), "jvrc1");
    solver().addConstraintSet(contactConstraint);
    solver().addConstraintSet(kinematicsConstraint);
    postureTask->stiffness(1);
    postureTask->weight(1);
    solver().addTask(postureTask.get());
    addContact({robot().name(), "ground", "LeftFoot", "AllGround"});
    addContact({robot().name(), "ground", "RightFoot", "AllGround"});

    /* Create and add the momentum task with the default stiffness/weight */
    pbvsTask = std::make_shared<mc_tasks::PositionBasedVisServoTask>(robot().frame("RightGripper"));
    pbvsTask->stiffness(10);
    solver().addTask(pbvsTask);

    mc_rtc::log::success("Created TestGazeTaskController");
  }

  virtual bool run() override
  {
    // Find and give the object position in camera frame
    auto X_0_s = robot().frame("RightGripper").position();
    auto X_t_s = X_0_s * object_.inv();
    pbvsTask->error(X_t_s);
    bool ret = MCController::run();
    if(!ret)
    {
      mc_rtc::log::critical("Failed at iter: {}", nrIter);
    }
    BOOST_CHECK(ret);
    nrIter++;
    if(nrIter == 500)
    {
      pbvsTask->stiffness(1000);
    }
    if(nrIter == 1000)
    {
      /* Check that the task is "finished" */
      BOOST_CHECK_SMALL(pbvsTask->eval().norm(), 1e-3);
      BOOST_CHECK_SMALL(pbvsTask->speed().norm(), 1e-2);
    }
    return ret;
  }

  virtual void reset(const ControllerResetData & reset_data) override
  {
    MCController::reset(reset_data);
    pbvsTask->reset();
    object_ = sva::PTransformd{sva::RotZ(3.14) * sva::RotY(1.57), Eigen::Vector3d{1.0, -0.25, 0.25}} * robot().posW();
    gui()->addElement({}, mc_rtc::gui::Transform("Camera", [this]() { return robot().frame("rcamera").position(); }),
                      mc_rtc::gui::Transform("Object", [this]() -> const sva::PTransformd & { return object_; }));
  }

private:
  unsigned int nrIter = 0;
  std::shared_ptr<mc_tasks::PositionBasedVisServoTask> pbvsTask = nullptr;
  /** Position of an object in the world */
  sva::PTransformd object_;
};

} // namespace mc_control

using Controller = mc_control::TestPBVSTaskController;
SIMPLE_CONTROLLER_CONSTRUCTOR("TestPBVSTaskController", Controller)
