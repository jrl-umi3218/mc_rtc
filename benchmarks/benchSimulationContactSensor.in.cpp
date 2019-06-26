/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/SimulationContactSensor.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>

#include "benchmark/benchmark.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wunknown-pragma"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"

mc_rbdyn::Robots & get_robots()
{
  static std::shared_ptr<mc_rbdyn::Robots> robots_ptr = nullptr;
  if(robots_ptr)
  {
    return *robots_ptr;
  }
  mc_rbdyn::RobotLoader::clear();
  mc_rbdyn::RobotLoader::update_robot_module_path({"@CMAKE_CURRENT_BINARY_DIR@/../src/mc_robots"});
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("HRP2DRC");
  auto env = mc_rbdyn::RobotLoader::get_robot_module("env", std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH),
                                                     std::string("ground"));
  robots_ptr = mc_rbdyn::loadRobots({rm, env});
  return *robots_ptr;
}

static void BM_Creation(benchmark::State & state)
{
  auto & robots = get_robots();
  auto & robot = robots.robot();
  auto & env = robots.env();
  while(state.KeepRunning())
  {
    mc_control::SimulationContactPair pair(robot.surfaces().at("LFullSole"), env.surfaces().at("AllGround"));
  }
}
BENCHMARK(BM_Creation)->Unit(benchmark::kMicrosecond);

static void BM_Update(benchmark::State & state)
{
  auto & robots = get_robots();
  auto & robot = robots.robot();
  auto & env = robots.env();

  mc_control::SimulationContactPair pair(robot.surfaces().at("LFullSole"), env.surfaces().at("AllGround"));

  while(state.KeepRunning())
  {
    robot.mbc().q[0].back() += 1e-6;
    robot.forwardKinematics();
    auto start = std::chrono::high_resolution_clock::now();
    pair.update(robot, env);
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
    state.SetIterationTime(elapsed_seconds.count());
  }
}
BENCHMARK(BM_Update);

BENCHMARK_MAIN();
