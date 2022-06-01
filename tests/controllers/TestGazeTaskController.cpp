/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifdef BOOST_TEST_MAIN
#  undef BOOST_TEST_MAIN
#endif

#include <mc_control/api.h>
#include <mc_control/mc_controller.h>

#include <mc_tasks/GazeTask.h>

#include <mc_rtc/logging.h>

#include <boost/test/unit_test.hpp>

namespace mc_control
{

struct MC_CONTROL_DLLAPI TestGazeTaskController : public MCController
{
public:
  TestGazeTaskController(mc_rbdyn::RobotModulePtr rm, double dt) : MCController(rm, dt)
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
    gazeTask = std::make_shared<mc_tasks::GazeTask>(robot().frame("rcamera"));
    gazeTask->stiffness(100);
    solver().addTask(gazeTask);

    mc_rtc::log::success("Created TestGazeTaskController");
  }

  virtual bool run() override
  {
    // Find and give the object position in camera frame
    auto X_0_gaze = robot().frame("rcamera").position();
    auto X_gaze_object = object_ * X_0_gaze.inv();
    gazeTask->error(X_gaze_object.translation());
    bool ret = MCController::run();
    if(!ret)
    {
      mc_rtc::log::critical("Failed at iter: {}", nrIter);
    }
    BOOST_CHECK(ret);
    nrIter++;
    if(nrIter == 500)
    {
      /* Check that the task is "finished" */
      BOOST_CHECK_SMALL(gazeTask->eval().norm(), 1e-3);
      BOOST_CHECK_SMALL(gazeTask->speed().norm(), 1e-4);
    }
    return ret;
  }

  virtual void reset(const ControllerResetData & reset_data) override
  {
    MCController::reset(reset_data);
    gazeTask->reset();
    object_ = sva::PTransformd{Eigen::Vector3d{1.0, 0.25, 0.25}} * robot().posW();
    gui()->addElement({}, mc_rtc::gui::Transform("Camera", [this]() { return robot().frame("rcamera").position(); }),
                      mc_rtc::gui::Transform("Object", [this]() -> const sva::PTransformd & { return object_; }));
  }

private:
  unsigned int nrIter = 0;
  std::shared_ptr<mc_tasks::GazeTask> gazeTask = nullptr;
  /** Position of an object in the world */
  sva::PTransformd object_;
};

} // namespace mc_control

using Controller = mc_control::TestGazeTaskController;
SIMPLE_CONTROLLER_CONSTRUCTOR("TestGazeTaskController", Controller)
