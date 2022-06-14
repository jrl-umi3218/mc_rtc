/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifdef BOOST_TEST_MAIN
#  undef BOOST_TEST_MAIN
#endif

#include <mc_control/api.h>
#include <mc_control/mc_controller.h>

#include <mc_rbdyn/RobotLoader.h>

#include <mc_solver/BoundedSpeedConstr.h>

#include <mc_rtc/logging.h>

#include <boost/test/unit_test.hpp>

#ifndef M_PI
#  include <boost/math/constants/constants.hpp>
#  define M_PI boost::math::constants::pi<double>()
#endif

namespace mc_control
{

/** The main purpose of this test is to test the collision constraint
 *
 * We spawn an additional box in the controller and move it towards the hand with multiple conditions:
 *
 * 1. Every joints are available to avoid the box
 *
 * 2. Only the arm joints can be used to avoid the collision
 *
 * 3. Only the shoulder pitch can be used to avoid the collision
 *
 * 4. All joints except the arm joints can be used
 *
 */
struct MC_CONTROL_DLLAPI TestCollisionController : public MCController
{
public:
  TestCollisionController(mc_rbdyn::RobotModulePtr rm, double dt)
  : MCController({rm, mc_rbdyn::RobotLoader::get_robot_module("env/ground"),
                  mc_rbdyn::RobotLoader::get_robot_module("object/box")},
                 dt),
    cstr_(robots(), robot("box").robotIndex(), dt)
  {
    // Check that the default constructor loads the robot + ground environment
    BOOST_REQUIRE_EQUAL(robots().size(), 3);
    // Check that JVRC-1 was loaded
    BOOST_REQUIRE_EQUAL(robot().name(), "jvrc1");
    // Check that the box is loaded
    BOOST_REQUIRE(robots().hasRobot("box"));

    solver().addConstraintSet(contactConstraint);
    solver().addConstraintSet(kinematicsConstraint);
    solver().addConstraintSet(cstr_);
    solver().addTask(postureTask);
    addContact({"jvrc1", "ground", "LeftFoot", "AllGround"});
    addContact({"jvrc1", "ground", "RightFoot", "AllGround"});
    addCollisions("jvrc1", "box", {{"R_WRIST_Y_S", "box", iDist, sDist, 0}});
    mc_rtc::log::success("Created TestCollisionController");
  }

  bool run() override
  {
    bool ret = MCController::run();
    BOOST_REQUIRE(ret);
    nrIter++;
    if(nrIter == 500)
    {
      resetBox();
      removeCollisions("jvrc1", "box");
      std::vector<std::string> arm_joints = {"R_SHOULDER_P", "R_SHOULDER_R", "R_SHOULDER_Y", "R_ELBOW_P",
                                             "R_ELBOW_Y",    "R_WRIST_R",    "R_WRIST_Y"};
      addCollisions("jvrc1", "box", {{"R_WRIST_Y_S", "box", iDist, sDist, 0, arm_joints}});
    }
    if(nrIter == 1000)
    {
      resetBox();
      removeCollisions("jvrc1", "box");
      std::vector<std::string> arm_joints = {"R_SHOULDER_P"};
      addCollisions("jvrc1", "box", {{"R_WRIST_Y_S", "box", iDist, sDist, 0, arm_joints}});
    }
    if(nrIter == 1500)
    {
      resetBox();
      removeCollisions("jvrc1", "box");
      std::vector<std::string> arm_joints = {"R_SHOULDER_P", "R_SHOULDER_R", "R_SHOULDER_Y", "R_ELBOW_P",
                                             "R_ELBOW_Y",    "R_WRIST_R",    "R_WRIST_Y"};
      std::vector<std::string> selected_joints = {"Root"};
      for(const auto & j : robot().module().ref_joint_order())
      {
        if(std::find(arm_joints.begin(), arm_joints.end(), j) != arm_joints.end())
        {
          selected_joints.push_back(j);
        }
      }
      addCollisions("jvrc1", "box", {{"R_WRIST_Y_S", "box", iDist, sDist, 0, selected_joints}});
    }
    return ret;
  }

  void reset(const ControllerResetData & reset_data) override
  {
    MCController::reset(reset_data);
    boxInitPos_ = {
        sva::RotZ(M_PI),
        (robot().frame("RightGripper").position() * sva::PTransformd{Eigen::Vector3d{0.5, 0, 0}}).translation()};
    resetBox();
    auto & box = robot("box");
    Eigen::MatrixXd dof = Eigen::Matrix6d::Identity();
    Eigen::Vector6d spd = Eigen::Vector6d::Zero();
    spd(3) = 0.5;
    cstr_.addBoundedSpeed(solver(), box.mb().body(0).name(), Eigen::Vector3d::Zero(), dof, spd);
  }

  void resetBox()
  {
    auto & box = robot("box");
    box.mbc().zero(box.mb());
    box.posW(boxInitPos_);
  }

private:
  unsigned int nrIter = 0;
  sva::PTransformd boxInitPos_;
  mc_solver::BoundedSpeedConstr cstr_;
  double iDist = 0.1;
  double sDist = 0.05;
};

} // namespace mc_control

SIMPLE_CONTROLLER_CONSTRUCTOR("TestCollisionController", mc_control::TestCollisionController)
