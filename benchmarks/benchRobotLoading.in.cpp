#include "benchmark/benchmark.h"

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>

static void BM_RobotModuleLoading(benchmark::State & state)
{
  mc_rbdyn::RobotLoader::clear();
  mc_rbdyn::RobotLoader::update_robot_module_path({"@CMAKE_CURRENT_BINARY_DIR@/../src/mc_robots"});
  while(state.KeepRunning())
  {
    auto rm = mc_rbdyn::RobotLoader::get_robot_module("HRP2DRC");
  }
}
BENCHMARK(BM_RobotModuleLoading)->Unit(benchmark::kMicrosecond);

static void BM_RobotCreation(benchmark::State & state)
{
  mc_rbdyn::RobotLoader::clear();
  mc_rbdyn::RobotLoader::update_robot_module_path({"@CMAKE_CURRENT_BINARY_DIR@/../src/mc_robots"});
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("HRP2DRC");
  while(state.KeepRunning())
  {
    auto robots = mc_rbdyn::loadRobot(*rm);
  }
}
BENCHMARK(BM_RobotCreation)->Unit(benchmark::kMicrosecond);

static void BM_RobotCopy(benchmark::State & state)
{
  mc_rbdyn::RobotLoader::clear();
  mc_rbdyn::RobotLoader::update_robot_module_path({"@CMAKE_CURRENT_BINARY_DIR@/../src/mc_robots"});
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("HRP2DRC");
  auto robots_ptr = mc_rbdyn::loadRobot(*rm);
  const auto & robots = *robots_ptr;
  while(state.KeepRunning())
  {
    auto robots_copy = robots;
  }
}
BENCHMARK(BM_RobotCopy)->Unit(benchmark::kMicrosecond);

BENCHMARK_MAIN();
