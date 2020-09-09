/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifdef BOOST_TEST_MAIN
#  undef BOOST_TEST_MAIN
#endif
#include <mc_control/api.h>
#include <mc_control/mc_controller.h>
#include <mc_observers/BodySensorObserver.h>
#include <mc_rtc/io_utils.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/CoMTask.h>

#include <boost/test/unit_test.hpp>
#include <utils.h>

namespace mc_control
{

struct MC_CONTROL_DLLAPI TestObserverController : public MCController
{
public:
  TestObserverController(std::shared_ptr<mc_rbdyn::RobotModule> rm, double dt) : MCController(rm, dt)
  {
    // Check that the default constructor loads the robot + ground environment
    BOOST_CHECK_EQUAL(robots().robots().size(), 2);
    // Load an additional main robot for the second pipeline
    loadRobot(rm, "jvrc1_2");
    BOOST_CHECK_EQUAL(robots().robots().size(), 3);
    // Check that JVRC-1 was loaded
    BOOST_CHECK_EQUAL(robot().name(), "jvrc1");
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

    mc_rtc::log::success("Created TestObserverController");
  }

  virtual bool run() override
  {
    // Check whether all pipelines succeeded
    for(const auto & pipeline : observerPipelines())
    {
      BOOST_REQUIRE(pipeline.success());
    }

    auto checkPose = [](const std::string & prefix, const sva::PTransformd & expected,
                        const sva::PTransformd & actual) {
      BOOST_CHECK_MESSAGE(allclose(expected.rotation(), actual.rotation(), 1e-6),
                          fmt::format("{} expected orientation [{}] but got [{}]", prefix,
                                      mc_rbdyn::rpyFromMat(expected.rotation()).transpose().format(Eigen::IOFormat(4)),
                                      mc_rbdyn::rpyFromMat(actual.rotation()).transpose().format(Eigen::IOFormat(4))));
      BOOST_CHECK_MESSAGE(allclose(expected.translation(), actual.translation(), 1e-6),
                          fmt::format("{} expected position [{}] but got [{}]", prefix,
                                      expected.translation().transpose(), actual.translation().transpose()));
    };

    // Check that the estimation from the KinematicInertial observers matches the
    // control robot state
    checkPose("KinematicInertial", robot().posW(), realRobot().posW());

    // Check that the estimation from the BodySensor observers matches the control robot state
    // This observer does not update the real robot instance, so to access its
    // result we need to access the actual BodySensorObserver object. This
    // implies explicitely linking the controller against it.
    auto & bodySensorObserver = observerPipeline().observer("BodySensor").observer<mc_observers::BodySensorObserver>();
    checkPose("BodySensor (no update)", robot().posW(), bodySensorObserver.posW());

    // Check that the second pipeline BodySensor observer works as expected
    checkPose("Second pipeline", robot().posW(), realRobot("jvrc1_2").posW());

    for(const auto joint : robot().refJointOrder())
    {
      auto j = robot().jointIndexByName(joint);
      BOOST_CHECK_CLOSE(robot().mbc().q[j][0], realRobot().mbc().q[j][0], 1e-6);
      BOOST_CHECK_CLOSE(robot().mbc().alpha[j][0], realRobot().mbc().alpha[j][0], 1e-6);
    }

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

    BOOST_REQUIRE_EQUAL(observerPipelines().size(), 2);
    BOOST_REQUIRE(hasObserverPipeline("FirstPipeline"));
    BOOST_REQUIRE(hasObserverPipeline("SecondPipeline"));
    // Check that the default pipeline is indeed the first one
    BOOST_REQUIRE_EQUAL(observerPipeline().name(), "FirstPipeline");

    const auto & pipelines = observerPipelines();
    BOOST_REQUIRE_EQUAL(pipelines[0].name(), "FirstPipeline");
    BOOST_REQUIRE_EQUAL(pipelines[1].name(), "SecondPipeline");

    const auto & firstPipeline = observerPipeline("FirstPipeline");
    BOOST_REQUIRE_EQUAL(firstPipeline.observers().size(), 4);
    BOOST_REQUIRE(firstPipeline.hasObserver("Encoder"));
    BOOST_REQUIRE(firstPipeline.hasObserver("Encoder2"));
    BOOST_REQUIRE(firstPipeline.hasObserver("BodySensor"));
    BOOST_REQUIRE(firstPipeline.hasObserver("KinematicInertial"));
    const auto & observers = firstPipeline.observers();
    // Check that order is respected
    BOOST_REQUIRE_EQUAL(observers[0].observer().name(), "Encoder");
    BOOST_REQUIRE_EQUAL(observers[0].update(), true);
    // Check that multiple observers of same type can be loaded
    BOOST_REQUIRE_EQUAL(observers[1].observer().name(), "Encoder2");
    BOOST_REQUIRE_EQUAL(observers[1].update(), false);
    BOOST_REQUIRE_EQUAL(observers[2].observer().name(), "BodySensor");
    BOOST_REQUIRE_EQUAL(observers[2].update(), false);
    BOOST_REQUIRE_EQUAL(observers[3].observer().name(), "KinematicInertial");
    BOOST_REQUIRE_EQUAL(observers[3].update(), true);
    BOOST_REQUIRE(firstPipeline.hasObserverType("Encoder"));
    BOOST_REQUIRE(firstPipeline.hasObserverType("BodySensor"));
    BOOST_REQUIRE(firstPipeline.hasObserverType("KinematicInertial"));

    // The pipelines haven't yet been exectued in the reset function
    BOOST_REQUIRE_EQUAL(firstPipeline.success(), false);

    const auto & secondPipeline = observerPipeline("SecondPipeline");
    BOOST_REQUIRE_EQUAL(secondPipeline.observers().size(), 3);
    BOOST_REQUIRE_EQUAL(secondPipeline.success(), false);
    BOOST_REQUIRE(secondPipeline.hasObserver("EncoderWithName"));
    BOOST_REQUIRE(secondPipeline.hasObserver("SecondRobotEncoder"));
    BOOST_REQUIRE(secondPipeline.hasObserver("SecondRobotBodySensor"));
    BOOST_REQUIRE(secondPipeline.hasObserverType("Encoder"));
    BOOST_REQUIRE(secondPipeline.hasObserverType("BodySensor"));
    BOOST_REQUIRE_EQUAL(secondPipeline.observer("EncoderWithName").update(), false);
    BOOST_REQUIRE_EQUAL(secondPipeline.observer("SecondRobotBodySensor").update(), true);
    BOOST_REQUIRE_EQUAL(secondPipeline.observer("SecondRobotEncoder").update(), true);

    // Check that the real robot was properly initialized
    BOOST_REQUIRE(allclose(realRobot().posW().translation(), robot().posW().translation()));
    BOOST_REQUIRE(allclose(realRobot().posW().rotation(), robot().posW().rotation()));

    // Add anchor frame
    datastore().make_call("KinematicAnchorFrame::" + robot().name(), [](const mc_rbdyn::Robot & robot) {
      return sva::interpolate(robot.surfacePose("LeftFoot"), robot.surfacePose("RightFoot"), 0.5);
    });
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
