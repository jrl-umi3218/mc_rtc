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
#include <mc_tasks/PositionTask.h>

#include <boost/test/unit_test.hpp>

namespace mc_control
{

struct MC_CONTROL_DLLAPI TestPositionTaskController : public MCController
{
public:
  TestPositionTaskController(std::shared_ptr<mc_rbdyn::RobotModule> rm, double dt) : MCController(rm, dt)
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
    solver().setContacts(
        {mc_rbdyn::Contact(robots(), "LeftFoot", "AllGround"), mc_rbdyn::Contact(robots(), "RightFoot", "AllGround")});

    /* Create and add the position task with the default stiffness/weight */
    posTask = std::make_shared<mc_tasks::PositionTask>("R_WRIST_Y_S", robots(), 0);
    posTask->stiffness(10);
    solver().addTask(posTask);

    comTask = std::make_shared<mc_tasks::CoMTask>(robots(), 0);
    solver().addTask(comTask);

    mc_rtc::log::success("Created TestPositionTaskController");
  }

  virtual bool run() override
  {
    bool ret = MCController::run();
    BOOST_CHECK(ret);
    nrIter++;
    if(nrIter == 1999)
    {
      /* Check that the task is "finished" */
      BOOST_CHECK_SMALL(posTask->eval().norm(), 1e-2);
      BOOST_CHECK_SMALL(posTask->speed().norm(), 1e-3);

      /* Apply dimWeight and give a "crazy" position target */
      posTask->dimWeight(Eigen::Vector3d(1., 1., 0.));
      posTask->position(posTask->position() + Eigen::Vector3d(0.1, 0, 100.0));
    }
    if(nrIter == 2999)
    {
      BOOST_CHECK_SMALL(posTask->eval().norm(), 1e-2);
      BOOST_CHECK_SMALL(posTask->speed().norm(), 1e-3);

      /* Reset the task and ask to raise the hand by 15 cm using only the
       * right arm joints */
      posTask->reset();
      posTask->dimWeight(Eigen::Vector3d(1., 1., 1.));
      posTask->selectActiveJoints(solver(), active_joints);
      posTask->position(posTask->position() + Eigen::Vector3d(0, 0, 0.15));
    }
    if(nrIter == 3999)
    {
      /* Check that the task is "finished" */
      BOOST_CHECK_SMALL(posTask->eval().norm(), 1e-2);
      BOOST_CHECK_SMALL(posTask->speed().norm(), 1e-2);

      /* Now move the hand down again, forbid rep movement in all tasks */
      posTask->reset();
      posTask->selectUnactiveJoints(solver(), {"R_ELBOW_P"});
      orig_rep = robot().mbc().q[robot().jointIndexByName("R_ELBOW_P")][0];
      posTask->position(posTask->position() + Eigen::Vector3d(0, 0, -0.15));

      comTask->selectUnactiveJoints(solver(), {"R_ELBOW_P"});

      /* Also reset the joint target in posture task */
      auto p = postureTask->posture();
      p[robot().jointIndexByName("R_ELBOW_P")][0] = orig_rep;
      postureTask->posture(p);
    }
    if(nrIter == 4999)
    {
      /* Check that the task is "finished" */
      BOOST_CHECK_SMALL(posTask->eval().norm(), 1e-2);
      BOOST_CHECK_SMALL(posTask->speed().norm(), 1e-2);

      /* And that R_ELBOW_P didn't move. Note that the error is not so
       * small because of other tasks' interaction */
      double current_rep = robot().mbc().q[robot().jointIndexByName("R_ELBOW_P")][0];
      BOOST_CHECK_SMALL(fabs(orig_rep - current_rep), 1e-2);
    }
    return ret;
  }

  virtual void reset(const ControllerResetData & reset_data) override
  {
    MCController::reset(reset_data);
    /* Reset the task to the current end-effector position */
    posTask->reset();
    comTask->reset();
    /* Move the end-effector 10cm forward, 10 cm to the right and 10 cm upward */
    posTask->position(posTask->position() + Eigen::Vector3d(0.3, -0.1, 0.2));
  }

private:
  unsigned int nrIter = 0;
  std::shared_ptr<mc_tasks::PositionTask> posTask = nullptr;
  std::shared_ptr<mc_tasks::CoMTask> comTask = nullptr;
  std::vector<std::string> active_joints = {"R_SHOULDER_P", "R_SHOULDER_R", "R_SHOULDER_Y", "R_ELBOW_P",
                                            "R_ELBOW_Y",    "R_WRIST_R",    "R_WRIST_Y"};
  double orig_rep = 0;
};

} // namespace mc_control

SIMPLE_CONTROLLER_CONSTRUCTOR("TestPositionTaskController", mc_control::TestPositionTaskController)
