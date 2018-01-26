#ifdef BOOST_TEST_MAIN
#undef BOOST_TEST_MAIN
#endif
#include <boost/test/unit_test.hpp>

#include <mc_control/mc_controller.h>
#include <mc_tasks/CoMTask.h>
#include <mc_control/api.h>

#include <mc_rtc/logging.h>

namespace mc_control
{

struct MC_CONTROL_DLLAPI TestCoMTaskController : public MCController
{
public:
  TestCoMTaskController(std::shared_ptr<mc_rbdyn::RobotModule> rm, double dt)
  : MCController(rm, dt)
  {
    // Check that the default constructor loads the robot + ground environment
    BOOST_CHECK_EQUAL(robots().robots().size(), 2);
    // Check that HRP2-DRC was loaded
    BOOST_CHECK_EQUAL(robot().name(), "hrp2_drc");
    solver().addConstraintSet(contactConstraint);
    solver().addConstraintSet(dynamicsConstraint);
    postureTask->stiffness(1);
    postureTask->weight(1);
    solver().addTask(postureTask.get());
    solver().setContacts({
      mc_rbdyn::Contact(robots(), "LFullSole", "AllGround"),
      mc_rbdyn::Contact(robots(), "RFullSole", "AllGround")
    });

    /* Create and add the CoM task with the default stiffness/weight */
    comTask = std::make_shared<mc_tasks::CoMTask>(robots(), 0);
    solver().addTask(comTask);

    LOG_SUCCESS("Created TestCoMTaskController")
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
      /* At this point the task error on z should be significant */
      BOOST_CHECK(fabs(comTask->eval().z()) > 99.99);
      /* But the task speed on z should be small */
      BOOST_CHECK_SMALL(fabs(comTask->speed().z()), 1e-2);

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

      /* Lower the CoM, forbid RLJ3 movement in all tasks */
      comTask->reset();
      comTask->selectUnactiveJoints(solver(), {"RLEG_JOINT3"});
      orig_rlj3 = robot().mbc().q[robot().jointIndexByName("RLEG_JOINT3")][0];
      comTask->com(comTask->com() + Eigen::Vector3d(0., 0., -0.05));

      /* Also reset the joint target in posture task */
      auto p = postureTask->posture();
      p[robot().jointIndexByName("RLEG_JOINT3")][0] = orig_rlj3;
      postureTask->posture(p);
      postureTask->jointStiffness(solver(), {{"RLEG_JOINT3", 1e5}});
    }
    if(nrIter == 4000)
    {
      /* Check that the task is "finished" */
      BOOST_CHECK_SMALL(comTask->speed().norm(), 1e-2);

      /* And that RLEG_JOINT3 didn't move. Note that the error is not so
       * small because of other tasks' interaction */
      double current_rlj3 = robot().mbc().q[robot().jointIndexByName("RLEG_JOINT3")][0];
      BOOST_CHECK_SMALL(fabs(orig_rlj3 - current_rlj3), 1e-2);
    }
    return ret;
  }

  virtual void reset(const ControllerResetData & reset_data) override
  {
    MCController::reset(reset_data);
    comTask->reset();
    /* Lower the CoM */
    comTask->com(comTask->com() + Eigen::Vector3d(0., 0., -0.05));
  }
private:
  unsigned int nrIter = 0;
  std::shared_ptr<mc_tasks::CoMTask> comTask = nullptr;
  std::vector<std::string> active_joints = [](){
    std::vector<std::string> ret;
    ret.push_back("Root");
    for(unsigned int i = 0; i < 6; ++i)
    {
      {
      std::stringstream ss;
      ss << "RLEG_JOINT" << i;
      ret.push_back(ss.str());
      }
      {
      std::stringstream ss;
      ss << "LLEG_JOINT" << i;
      ret.push_back(ss.str());
      }
    }
    return ret;
  }();
  double orig_rlj3 = 0;
};

}

SIMPLE_CONTROLLER_CONSTRUCTOR("TestCoMTaskController", mc_control::TestCoMTaskController)
