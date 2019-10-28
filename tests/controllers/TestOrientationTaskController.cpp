/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifdef BOOST_TEST_MAIN
#  undef BOOST_TEST_MAIN
#endif
#include <mc_control/api.h>
#include <mc_control/mc_controller.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/OrientationTask.h>

#include <boost/test/unit_test.hpp>

#ifndef M_PI
#  include <boost/math/constants/constants.hpp>
#  define M_PI boost::math::constants::pi<double>()
#endif

namespace mc_control
{

struct MC_CONTROL_DLLAPI TestOrientationTaskController : public MCController
{
public:
  TestOrientationTaskController(std::shared_ptr<mc_rbdyn::RobotModule> rm, double dt) : MCController(rm, dt)
  {
    // Check that the default constructor loads the robot + ground environment
    BOOST_CHECK_EQUAL(robots().robots().size(), 2);
    // Check that JVRC-1 was loaded
    BOOST_CHECK_EQUAL(robot().name(), "jvrc1");
    solver().addConstraintSet(contactConstraint);
    solver().addConstraintSet(dynamicsConstraint);
    postureTask->stiffness(1);
    postureTask->weight(1);
    solver().addTask(postureTask.get());
    solver().setContacts(
        {mc_rbdyn::Contact(robots(), "LeftFoot", "AllGround"), mc_rbdyn::Contact(robots(), "RightFoot", "AllGround")});

    /* Create and add the position task with the default stiffness/weight */
    oriTask = std::make_shared<mc_tasks::OrientationTask>("R_WRIST_Y_S", robots(), 0);
    solver().addTask(oriTask);

    comTask = std::make_shared<mc_tasks::CoMTask>(robots(), 0);
    solver().addTask(comTask);

    LOG_SUCCESS("Created TestOrientationTaskController")
  }

  virtual bool run() override
  {
    bool ret = MCController::run();
    BOOST_CHECK(ret);
    nrIter++;
    if(nrIter == 1500)
    {
      /* Check that the task is "finished" */
      BOOST_CHECK_SMALL(oriTask->eval().norm(), 1e-3);
      BOOST_CHECK_SMALL(oriTask->speed().norm(), 1e-3);

      /* Apply dimWeight to ignore rotation around z */
      oriTask->dimWeight(Eigen::Vector3d(1., 1., 0.));
      oriTask->orientation(oriTask->orientation() * sva::RotZ<double>(-M_PI / 2));
    }
    if(nrIter == 3000)
    {
      /* At this point the task error on z should be significant */
      BOOST_CHECK(fabs(oriTask->eval().z() - M_PI / 2) > 0.1);
      /* But the task speed on z should be small */
      BOOST_CHECK_SMALL(fabs(oriTask->speed().z()), 1e-2);

      /* Reset the task and ask to raise the hand by 15 cm using only the
       * right arm joints */
      oriTask->reset();
      oriTask->dimWeight(Eigen::Vector3d(1., 1., 1.));
      oriTask->selectActiveJoints(solver(), active_joints);
      oriTask->orientation(oriTask->orientation() * sva::RotZ<double>(-M_PI / 2));
    }
    if(nrIter == 4500)
    {
      /* Check that the task is "finished" */
      BOOST_CHECK_SMALL(oriTask->eval().norm(), 1e-2);
      BOOST_CHECK_SMALL(oriTask->speed().norm(), 1e-2);

      /* Now move the hand down again, forbid elbow pitch movement */
      oriTask->reset();
      oriTask->selectUnactiveJoints(solver(), {"R_ELBOW_P"});
      orig_rep = robot().mbc().q[robot().jointIndexByName("R_ELBOW_P")][0];
      oriTask->orientation(oriTask->orientation() * sva::RotZ<double>(M_PI / 2));

      comTask->selectUnactiveJoints(solver(), {"R_ELBOW_P"});

      /* Also reset the joint target in posture task */
      auto p = postureTask->posture();
      p[robot().jointIndexByName("R_ELBOW_P")][0] = orig_rep;
      postureTask->posture(p);
    }
    if(nrIter == 6000)
    {
      /* Check that the task is "finished" */
      BOOST_CHECK_SMALL(oriTask->eval().norm(), 1e-2);
      BOOST_CHECK_SMALL(oriTask->speed().norm(), 1e-2);

      /* And that R_ELBOW_P didn't move. Note that the error is not so
       * small because of other tasks' interaction */
      double current_rep = robot().mbc().q[robot().jointIndexByName("R_ELBOW_P")][0];
      BOOST_CHECK_SMALL(fabs(orig_rep - current_rep), 5e-2);
    }
    return ret;
  }

  virtual void reset(const ControllerResetData & reset_data) override
  {
    MCController::reset(reset_data);
    /* Reset the task to the current end-effector position */
    oriTask->reset();
    comTask->reset();
    /* Align the wrist X axis with gravity */
    oriTask->orientation(sva::RotY(-M_PI / 2));
  }

private:
  unsigned int nrIter = 0;
  std::shared_ptr<mc_tasks::OrientationTask> oriTask = nullptr;
  std::shared_ptr<mc_tasks::CoMTask> comTask = nullptr;
  std::vector<std::string> active_joints = {"R_SHOULDER_P", "R_SHOULDER_R", "R_SHOULDER_Y", "R_ELBOW_P",
                                            "R_ELBOW_Y",    "R_WRIST_R",    "R_WRIST_Y"};
  double orig_rep = 0;
};

} // namespace mc_control

SIMPLE_CONTROLLER_CONSTRUCTOR("TestOrientationTaskController", mc_control::TestOrientationTaskController)
