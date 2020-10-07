/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>
#include <mc_solver/QPSolver.h>
#include <mc_tasks/CoMTask.h>

#include <boost/test/unit_test.hpp>

#include "utils.h"

/** This test verifies that task's life is properly extended by the solver */

/** Normal CoMTask with static boolean to check the object is deleted */
struct CheckCoMTask : public mc_tasks::CoMTask
{
  CheckCoMTask(const mc_rbdyn::Robots & robots, unsigned int robotIndex) : mc_tasks::CoMTask(robots, robotIndex) {}
  virtual ~CheckCoMTask() override
  {
    deleted = true;
  }
  static bool deleted;
};

bool CheckCoMTask::deleted = false;

BOOST_AUTO_TEST_CASE(TestSolverTaskStorage)
{
  configureRobotLoader();
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
  auto robots = mc_rbdyn::loadRobot(*rm);
  mc_solver::QPSolver solver(robots, 0.005);
  CheckCoMTask * t_ptr = nullptr;
  {
    auto t = std::make_shared<CheckCoMTask>(solver.robots(), 0);
    t_ptr = t.get();
    solver.addTask(t);
  }
  BOOST_REQUIRE(CheckCoMTask::deleted == false);
  solver.removeTask(t_ptr);
  BOOST_REQUIRE(CheckCoMTask::deleted == true);
}
