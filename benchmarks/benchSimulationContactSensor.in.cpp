/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/SimulationContactPair.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>

#include <spdlog/spdlog.h>

#include "benchmark/benchmark.h"

class SimulationContactPairFixture : public benchmark::Fixture
{
public:
  void SetUp(const ::benchmark::State &)
  {
    spdlog::set_level(spdlog::level::err);
    mc_rbdyn::RobotLoader::clear();
    mc_rbdyn::RobotLoader::update_robot_module_path({"@CMAKE_CURRENT_BINARY_DIR@/../src/mc_robots"});
    auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
    auto env = mc_rbdyn::RobotLoader::get_robot_module("env", std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH),
                                                       std::string("ground"));
    robots_ = mc_rbdyn::loadRobots({rm, env});
  }

  void TearDown(const ::benchmark::State &) {}

  mc_rbdyn::Robots & get_robots()
  {
    return *robots_;
  }

  std::shared_ptr<mc_rbdyn::Robots> robots_;
};

BENCHMARK_DEFINE_F(SimulationContactPairFixture, Creation)(benchmark::State & state)
{
  auto & robots = get_robots();
  auto & robot = robots.robot();
  auto & env = robots.env();
  while(state.KeepRunning())
  {
    mc_control::SimulationContactPair pair(robot.surfaces().at("LeftFoot"), env.surfaces().at("AllGround"));
  }
}
BENCHMARK_REGISTER_F(SimulationContactPairFixture, Creation)->Unit(benchmark::kMicrosecond);

BENCHMARK_DEFINE_F(SimulationContactPairFixture, Update)(benchmark::State & state)
{
  auto & robots = get_robots();
  auto & robot = robots.robot();
  auto & env = robots.env();

  mc_control::SimulationContactPair pair(robot.surfaces().at("LeftFoot"), env.surfaces().at("AllGround"));

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
BENCHMARK_REGISTER_F(SimulationContactPairFixture, Update)->Unit(benchmark::kMicrosecond);

BENCHMARK_MAIN();
