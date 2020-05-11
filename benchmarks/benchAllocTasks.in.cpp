#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>
#include <mc_solver/QPSolver.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/SurfaceTransformTask.h>
#include <mc_tasks/lipm_stabilizer/StabilizerTask.h>

#include "benchmark/benchmark.h"

static void BM_AllocSurfaceTransformTask(benchmark::State & state)
{
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
  mc_rbdyn::Robots robots;
  robots.load(*rm);
  for(auto _ : state)
  {
    auto task = std::make_shared<mc_tasks::SurfaceTransformTask>("LeftFoot", robots, 0);
  }
}

BENCHMARK(BM_AllocSurfaceTransformTask);

static void BM_SurfaceTransformTaskFromConfig(benchmark::State & state)
{
  mc_rtc::Configuration config("@CMAKE_CURRENT_SOURCE_DIR@/config.yaml");
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
  auto robots = mc_rbdyn::loadRobot(*rm);
  mc_solver::QPSolver solver{robots, 0.005};
  for(auto _ : state)
  {
    auto task = mc_tasks::MetaTaskLoader::load<mc_tasks::SurfaceTransformTask>(solver, config);
  }
}

BENCHMARK(BM_SurfaceTransformTaskFromConfig);

static void BM_AllocStabilizerTask(benchmark::State & state)
{
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
  auto robots = mc_rbdyn::loadRobot(*rm);
  mc_solver::QPSolver solver{robots, 0.005};
  for(auto _ : state)
  {
    auto task = std::make_shared<mc_tasks::lipm_stabilizer::StabilizerTask>(*robots, *robots, 0, "LeftFoot",
                                                                            "RightFoot", "WAIST_R_S", solver.dt());
  }
}

BENCHMARK(BM_AllocStabilizerTask);

static void BM_StabilizerTaskFromConfig(benchmark::State & state)
{
  mc_rtc::Configuration config("@CMAKE_CURRENT_SOURCE_DIR@/config_lipm.yaml");
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
  auto robots = mc_rbdyn::loadRobot(*rm);
  mc_solver::QPSolver solver{robots, 0.005};
  solver.realRobots(robots);
  for(auto _ : state)
  {
    auto task = mc_tasks::MetaTaskLoader::load<mc_tasks::lipm_stabilizer::StabilizerTask>(solver, config);
  }
}

BENCHMARK(BM_StabilizerTaskFromConfig);

BENCHMARK_MAIN()
