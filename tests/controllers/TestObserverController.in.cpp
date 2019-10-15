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

#include <boost/test/unit_test.hpp>

namespace mc_control
{

struct MC_CONTROL_DLLAPI TestObserverController : public MCController
{
public:
  TestObserverController(std::shared_ptr<mc_rbdyn::RobotModule> rm, double dt) : MCController(rm, dt)
  {
    // Check that the default constructor loads the robot + ground environment
    BOOST_CHECK_EQUAL(robots().robots().size(), 2);
    // Check that JVRC-1 was loaded
    BOOST_CHECK_EQUAL(robot().name(), "jvrc-1");
    solver().addConstraintSet(contactConstraint);
    solver().addConstraintSet(dynamicsConstraint);
    postureTask->stiffness(1);
    postureTask->weight(1);
    solver().addTask(postureTask.get());
    solver().setContacts(
        {mc_rbdyn::Contact(robots(), "LeftFoot", "AllGround"), mc_rbdyn::Contact(robots(), "RightFoot", "AllGround")});

    /* Create and add the CoM task with the default stiffness/weight */
    comTask = std::make_shared<mc_tasks::CoMTask>(robots(), 0);
    comTask->stiffness(10);
    solver().addTask(comTask);

    LOG_SUCCESS("Created TestObserverController")
  }

  virtual bool run() override
  {
    bool ret = MCController::run();
    BOOST_CHECK(ret);
    nrIter++;
    if(nrIter == 1000)
    {
      /* Check that the task is "finished" */
      BOOST_CHECK_SMALL(comTask->eval().norm(), 1e-2);
      BOOST_CHECK_SMALL(comTask->speed().norm(), 1e-3);

      /* Apply dimWeight and give a "crazy" position target */
      comTask->dimWeight(Eigen::Vector3d(1., 1., 0.));
      comTask->move_com(Eigen::Vector3d(0., 0., 100.));
      postureTask->posture(robot().mbc().q);
    }
    if(nrIter == 2000)
    {
      BOOST_CHECK_SMALL(comTask->eval().norm(), 1e-2);
      BOOST_CHECK_SMALL(comTask->speed().norm(), 1e-3);

      /* Raise the CoM again, using only the leg joints */
      comTask->reset();
      comTask->dimWeight(Eigen::Vector3d(1., 1., 1.));
      comTask->selectActiveJoints(solver(), active_joints);
      comTask->com(comTask->com() + Eigen::Vector3d(0., 0., 0.05));
    }
    if(nrIter == 3000)
    {
      /* Check that the task is "finished" */
      BOOST_CHECK_SMALL(comTask->eval().norm(), 2e-2);
      BOOST_CHECK_SMALL(comTask->speed().norm(), 1e-2);

      /* Lower the CoM, forbid right knee movement in all tasks */
      comTask->reset();
      comTask->selectUnactiveJoints(solver(), {"R_KNEE"});
      orig_rkj = robot().mbc().q[robot().jointIndexByName("R_KNEE")][0];
      comTask->com(comTask->com() + Eigen::Vector3d(0., 0., -0.05));

      /* Also reset the joint target in posture task */
      postureTask->reset();
      postureTask->jointStiffness(solver(), {{"R_KNEE", 1e5}});
    }
    if(nrIter == 4000)
    {
      /* Check that the task is "finished" */
      BOOST_CHECK_SMALL(comTask->speed().norm(), 1e-2);

      /* And that RLEG_JOINT3 didn't move. Note that the error is not so
       * small because of other tasks' interaction */
      double current_rkj = robot().mbc().q[robot().jointIndexByName("R_KNEE")][0];
      BOOST_CHECK_SMALL(fabs(orig_rkj - current_rkj), 1e-2);
    }
    return ret;
  }

  virtual void reset(const ControllerResetData & reset_data) override
  {
    MCController::reset(reset_data);
    comTask->reset();
    /* Lower the CoM */
    comTask->com(comTask->com() + Eigen::Vector3d(0., 0., -0.05));

    // clang-format off
    std::vector<std::string> enabledObservers = { @ENABLED_OBSERVERS@ };
    std::vector<std::string> runObservers = { @RUN_OBSERVERS@ };
    std::vector<std::string> updateObservers = { @UPDATE_OBSERVERS@ };
    // clang-format on

    auto hasObserver = [this](const std::string & name) {
      for(const auto & obs : observers_)
      {
        if(obs->name() == name)
        {
          return true;
        }
      }
      return false;
    };
    auto hasPipelineObserver = [this](const std::string & name, bool update) {
      for(const auto & obs : pipelineObservers_)
      {
        if(obs.first->name() == name)
        {
          return update ? obs.second : true;
        }
      }
      return false;
    };
    for(const auto & enabled : enabledObservers)
    {
      BOOST_REQUIRE(hasObserver(enabled));
    }
    for(const auto & run : runObservers)
    {
      BOOST_REQUIRE(hasPipelineObserver(run, false));
    }
    for(const auto & update : updateObservers)
    {
      BOOST_REQUIRE(hasPipelineObserver(update, false));
    }
  }

private:
  unsigned int nrIter = 0;
  std::shared_ptr<mc_tasks::CoMTask> comTask = nullptr;
  std::vector<std::string> active_joints = {"Root",      "R_HIP_P",   "R_HIP_R",  "R_HIP_Y", "R_KNEE",
                                            "R_ANKLE_R", "R_ANKLE_P", "L_HIP_P",  "L_HIP_R", "L_HIP_Y",
                                            "L_KNEE",    "L_ANKLE_R", "L_ANKLE_P"};
  double orig_rkj = 0;
};

} // namespace mc_control

SIMPLE_CONTROLLER_CONSTRUCTOR("TestObserverController", mc_control::TestObserverController)
