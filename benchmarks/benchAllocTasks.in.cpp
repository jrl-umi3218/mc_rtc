#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>
#include <mc_solver/QPSolver.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/SurfaceTransformTask.h>
#include <mc_tasks/lipm_stabilizer/StabilizerTask.h>

#include "benchmark/benchmark.h"

class AllocTaskFixture : public benchmark::Fixture
{
public:
  void SetUp(const ::benchmark::State &)
  {
    // XXX silence error output
    std::cerr.rdbuf(nullptr);
    auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
    solver.robots().load(*rm);
    solver.realRobots(std::make_shared<mc_rbdyn::Robots>(solver.robots()));
  }

  void TearDown(const ::benchmark::State &) {}

  mc_solver::QPSolver solver{0.005};
};

BENCHMARK_F(AllocTaskFixture, AllocSurfaceTransformTask)(benchmark::State & state)
{
  for(auto _ : state)
  {
    auto task = std::make_shared<mc_tasks::SurfaceTransformTask>("LeftFoot", solver.robots(), 0);
  }
}

BENCHMARK_F(AllocTaskFixture, SurfaceTransformTaskFromConfig)(benchmark::State & state)
{
  mc_rtc::Configuration config("@CMAKE_CURRENT_SOURCE_DIR@/config.yaml");
  for(auto _ : state)
  {
    auto task = mc_tasks::MetaTaskLoader::load<mc_tasks::SurfaceTransformTask>(solver, config);
  }
}

BENCHMARK_F(AllocTaskFixture, StabilizerTask)(benchmark::State & state)
{
  for(auto _ : state)
  {
    auto & robots = solver.robots();
    auto task = std::make_shared<mc_tasks::lipm_stabilizer::StabilizerTask>(robots, robots, 0, "LeftFoot", "RightFoot",
                                                                            "WAIST_R_S", solver.dt());
  }
}

BENCHMARK_F(AllocTaskFixture, StabilizerTaskFromConfig)(benchmark::State & state)
{
  mc_rtc::Configuration config("@CMAKE_CURRENT_SOURCE_DIR@/config_lipm.yaml");
  for(auto _ : state)
  {
    auto task = mc_tasks::MetaTaskLoader::load<mc_tasks::lipm_stabilizer::StabilizerTask>(solver, config);
  }
}

BENCHMARK_MAIN();
