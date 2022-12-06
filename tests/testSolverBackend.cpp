/*
 * Copyright 2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_solver/ConstraintSetLoader.h>
#include <mc_solver/QPSolver.h>
#include <mc_solver/TasksQPSolver.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <boost/test/unit_test.hpp>

#include <thread>

#include "utils.h"

BOOST_AUTO_TEST_CASE(TestSolverBackend)
{
  [[maybe_unused]] bool configured = configureRobotLoader();
  /* Create Robots with one robot and an environment for the purpose of the test */
  static auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
  static auto em = mc_rbdyn::RobotLoader::get_robot_module("env/ground");
  static auto robots = mc_rbdyn::loadRobotAndEnv(*rm, *em);
  // Checks that creating a solver sets the backend on this thread
  BOOST_REQUIRE(mc_solver::QPSolver::context_backend() == mc_solver::QPSolver::Backend::Unset);
  auto solver = mc_solver::TasksQPSolver(robots, 0.005);
  BOOST_REQUIRE(mc_solver::QPSolver::context_backend() == mc_solver::QPSolver::Backend::Tasks);
  {
    // Checks that on a new thread the backend is unset and can be set
    std::thread th([&solver]() {
      BOOST_REQUIRE(mc_solver::QPSolver::context_backend() == mc_solver::QPSolver::Backend::Unset);
      mc_solver::QPSolver::context_backend(solver.backend());
      BOOST_REQUIRE(mc_solver::QPSolver::context_backend() == mc_solver::QPSolver::Backend::Tasks);
    });
    th.join();
  }
  {
    // Check that MetaTaskLoader automatically sets the context backend
    std::thread th([&solver]() {
      BOOST_REQUIRE(mc_solver::QPSolver::context_backend() == mc_solver::QPSolver::Backend::Unset);
      mc_rtc::Configuration config;
      config.add("type", "com");
      auto task = mc_tasks::MetaTaskLoader::load(solver, config);
      BOOST_REQUIRE(task != nullptr);
      BOOST_REQUIRE(mc_solver::QPSolver::context_backend() == mc_solver::QPSolver::Backend::Tasks);
    });
    th.join();
  }
  {
    // Check that ConstraintSetLoader automatically sets the context backend
    std::thread th([&solver]() {
      BOOST_REQUIRE(mc_solver::QPSolver::context_backend() == mc_solver::QPSolver::Backend::Unset);
      mc_rtc::Configuration config;
      config.add("type", "contact");
      auto cstr = mc_solver::ConstraintSetLoader::load(solver, config);
      BOOST_REQUIRE(cstr != nullptr);
      BOOST_REQUIRE(mc_solver::QPSolver::context_backend() == mc_solver::QPSolver::Backend::Tasks);
    });
    th.join();
  }
}
