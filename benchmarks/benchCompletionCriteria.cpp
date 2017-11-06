#include "benchmark/benchmark.h"

#include <mc_control/mc_completion_criteria.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_tasks/CoMTask.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wunknown-pragma"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"

const double dt = 0.005;

mc_rbdyn::Robots & get_robots()
{
  static std::shared_ptr<mc_rbdyn::Robots> robots_ptr = nullptr;
  if(robots_ptr)
  {
    return *robots_ptr;
  }
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("HRP2DRC");
  robots_ptr = mc_rbdyn::loadRobot(*rm);
  return *robots_ptr;
}

struct MockTask : public mc_tasks::CoMTask
{
  MockTask() : mc_tasks::CoMTask(get_robots(), 0) {}

  Eigen::VectorXd eval() const override { return eval_; }
  Eigen::VectorXd speed() const override { return speed_; }

  Eigen::Vector3d eval_ = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d speed_ = Eigen::Vector3d::UnitZ();
};

static void BM_DirectTimeout(benchmark::State & state)
{
  unsigned int tick = 0;
  double timeout = 5.0;
  unsigned int target = std::ceil(timeout / dt);
  bool b;
  std::string o;
  while(state.KeepRunning())
  {
    o = "";
    if(++tick > target)
    {
      b = true;
      o += "timeout";
    }
    b = false;
  }
}
BENCHMARK(BM_DirectTimeout);

static void BM_Timeout(benchmark::State & state)
{
  MockTask task;
  double timeout = 5.0;
  mc_rtc::Configuration config;
  config.add("timeout", timeout);
  mc_control::CompletionCriteria criteria;
  criteria.configure(dt, config);
  bool b;
  while(state.KeepRunning())
  {
    b = criteria.completed(task);
  }
}
BENCHMARK(BM_Timeout);

static void BM_DirectEvalAndSpeedOrTimeout(benchmark::State & state)
{
  unsigned int tick = 0;
  double timeout = 5.0;
  unsigned int target = std::ceil(timeout / dt);
  bool b;
  std::string o;
  double norm = 1e-3;
  MockTask task;
  while(state.KeepRunning())
  {
    o = "";
    if(task.eval().norm() < norm && task.speed().norm() < norm)
    {
      b = true;
      o += "eval AND speed";
    }
    else if(++tick > target)
    {
      b = true;
      o += "timeout";
    }
    else
    {
      b = false;
    }
  }
}
BENCHMARK(BM_DirectEvalAndSpeedOrTimeout);

static void BM_EvalAndSpeedOrTimeout(benchmark::State & state)
{
  bool b;
  MockTask task;
  double norm = 1e-3;
  double timeout = 5.0;
  mc_rtc::Configuration config;
  auto OR = config.array("OR", 2);
  OR.push([norm](){
          mc_rtc::Configuration c;
          auto AND = c.array("AND", 2);
          AND.push([norm](){ mc_rtc::Configuration c; c.add("eval", norm); return c; }());
          AND.push([norm](){ mc_rtc::Configuration c; c.add("speed", norm); return c; }());
          return c;
          }());
  OR.push([timeout](){ mc_rtc::Configuration c; c.add("timeout", timeout); return c; }());
  mc_control::CompletionCriteria criteria;
  criteria.configure(dt, config);
  while(state.KeepRunning())
  {
    b = criteria.completed(task);
  }
}
BENCHMARK(BM_EvalAndSpeedOrTimeout);

static void BM_TimeoutConfigure(benchmark::State & state)
{
  MockTask task;
  double timeout = 5.0;
  mc_rtc::Configuration config;
  config.add("timeout", timeout);
  mc_control::CompletionCriteria criteria;
  while(state.KeepRunning())
  {
    criteria.configure(dt, config);
  }
}
BENCHMARK(BM_TimeoutConfigure);

static void BM_EvalAndSpeedOrTimeoutConfigure(benchmark::State & state)
{
  MockTask task;
  double norm = 1e-3;
  double timeout = 5.0;
  mc_rtc::Configuration config;
  auto OR = config.array("OR", 2);
  OR.push([norm](){
          mc_rtc::Configuration c;
          auto AND = c.array("AND", 2);
          AND.push([norm](){ mc_rtc::Configuration c; c.add("eval", norm); return c; }());
          AND.push([norm](){ mc_rtc::Configuration c; c.add("speed", norm); return c; }());
          return c;
          }());
  OR.push([timeout](){ mc_rtc::Configuration c; c.add("timeout", timeout); return c; }());
  mc_control::CompletionCriteria criteria;
  while(state.KeepRunning())
  {
    criteria.configure(dt, config);
  }
}
BENCHMARK(BM_EvalAndSpeedOrTimeoutConfigure);

BENCHMARK_MAIN();
