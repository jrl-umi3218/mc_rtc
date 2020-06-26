/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_planning/LookupTable.h>
#include <mc_planning/generator.h>
#include <mc_rtc/constants.h>
#include "benchmark/benchmark.h"
#include <random>

static void BM_cosh_lookuptable(benchmark::State & state)
{
  // Perform setup here
  auto omega = [](double h) { return std::sqrt(mc_rtc::constants::GRAVITY / h); };
  double dt = 0.005;
  auto table =
      mc_planning::LookupTable<double>(20000, omega(2.5), omega(0.01), [dt](double x) { return cosh(x * dt); });

  static std::random_device rd;
  static std::mt19937_64 gen(rd());
  static std::uniform_real_distribution<double> dis(omega(2.5), omega(0.01));

  double res = 0;
  double val = 0;
  for(auto _ : state)
  {
    val = dis(gen);
    for(int i = 0; i < 10000; ++i)
    {
      // Prevent compiler from optimizing away the result
      benchmark::DoNotOptimize(res = table(val));
    }
  }
  state.SetItemsProcessed(10000 * state.iterations());
}
BENCHMARK(BM_cosh_lookuptable);

static void BM_cosh(benchmark::State & state)
{
  // Perform setup here
  auto omega = [](double h) { return std::sqrt(mc_rtc::constants::GRAVITY / h); };
  double dt = 0.005;
  static std::random_device rd;
  static std::mt19937_64 gen(rd());
  static std::uniform_real_distribution<double> dis(omega(2.5), omega(0.01));

  double res = 0;
  double val = 0;
  for(auto _ : state)
  {
    val = dis(gen);
    for(int i = 0; i < 10000; ++i)
    {
      benchmark::DoNotOptimize(res = cosh(val * dt));
    }
  }
  state.SetItemsProcessed(10000 * state.iterations());
}
BENCHMARK(BM_cosh);

class PreviewStepsFixture : public benchmark::Fixture
{
  using PreviewSteps = mc_planning::PreviewSteps<Eigen::Vector2d>;

public:
  void SetUp(const ::benchmark::State &)
  {
    steps = std::make_shared<PreviewSteps>();
    steps->add({0, {0.0, 0.0}});
    steps->add({1.5, {0.0, 1.5}});
    steps->add({1.6, {0.0, 1.6}});
    steps->add({2.9, {0.0, 2.9}});
    steps->add({3.0, {0.0, 3.0}});
    steps->add({5.0, {0.0, 3.0}});
    steps->add({10.0, {0.0, 3.0}});
    steps->initialize();
  }

  void TearDown(const ::benchmark::State &) {}

  static constexpr unsigned nIter()
  {
    return n_loop * (2 * n_preview + 1);
  }

  std::shared_ptr<PreviewSteps> steps;
  static constexpr double dt = 0.005;
  static constexpr unsigned n_loop = 15. / dt;
  static constexpr unsigned n_preview = 1.6 / dt;
};

BENCHMARK_DEFINE_F(PreviewStepsFixture, BM_PreviewSteps)(benchmark::State & state)
{
  while(state.KeepRunning())
  {
    for(unsigned loop = 0; loop <= n_loop; loop++)
    {
      auto previewStart = loop * dt;
      steps->nextWindow(previewStart);

      for(unsigned previewWindow = loop; previewWindow < loop + 2 * n_preview + 1; ++previewWindow)
      {
        double t = previewWindow * dt;
        steps->update(t);

        if(!steps->isLastStep())
        {
          mc_planning::TimedStep<Eigen::Vector2d> prev, next;
          benchmark::DoNotOptimize(prev = steps->previous());
          benchmark::DoNotOptimize(next = steps->next());
        }
      }
    }
  }
  state.SetItemsProcessed(PreviewStepsFixture::nIter() * state.iterations());
}
BENCHMARK_REGISTER_F(PreviewStepsFixture, BM_PreviewSteps)->Unit(benchmark::kMicrosecond);

class GeneratorFixture : public benchmark::Fixture
{
public:
  using Generator = mc_planning::generator;
  using PreviewSteps = mc_planning::PreviewSteps<Eigen::Vector2d>;
  using CenteredPreviewWindow = mc_planning::CenteredPreviewWindow;
  GeneratorFixture() : preview(1.6, dt) {}

  void SetUp(const ::benchmark::State &)
  {
    steps.add({preview.halfDuration(), {-0.2, 0.0}});
    steps.addRelative({preview.halfDuration(), {0.0, 0.0}});
    steps.addRelative({0.1, {0.0, 0.0}});
    steps.addRelative({1.6, {0.0, 0.0}});
    steps.addRelative({0.1, {0.2, 0.095}});
    steps.addRelative({1.6, {0.0, 0.0}});
    steps.addRelative({0.1, {0.0, -0.19}});
    steps.addRelative({1.6, {0.0, 0.0}});
    steps.addRelative({0.1, {-0.2, 0.095}});
    steps.addRelative({preview.halfDuration(), {0.0, 0.0}});
    steps.initialize();
    generator = std::make_shared<Generator>(preview);
    generator->steps(steps);

    n_loop = preview.indexFromTime(steps.back().t());
  }

  void TearDown(const ::benchmark::State &) {}

  std::shared_ptr<Generator> generator;
  CenteredPreviewWindow preview;
  PreviewSteps steps;
  static constexpr double dt = 0.005;
  unsigned n_loop = 0.;
  static constexpr unsigned n_preview = 1.6 / dt;
};

BENCHMARK_DEFINE_F(GeneratorFixture, BM_Generator)(benchmark::State & state)
{
  for(auto _ : state)
  {
    for(unsigned loop = 0; loop <= n_loop; loop++)
    {
      generator->generate(loop);
    }
  }
  state.SetItemsProcessed(n_loop * state.iterations());
}
BENCHMARK_REGISTER_F(GeneratorFixture, BM_Generator)->Unit(benchmark::kMicrosecond);

// Run the benchmark
BENCHMARK_MAIN();
