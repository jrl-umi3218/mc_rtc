/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_planning/LookupTable.h>
#include <mc_rtc/constants.h>
#include "benchmark/benchmark.h"
#include <random>

static void BM_cosh_lookuptable(benchmark::State & state)
{
  // Perform setup here
  auto omega = [](double h) { return std::sqrt(mc_rtc::constants::GRAVITY / h); };
  double dt = 0.005;
  auto table =
      mc_planning::LookupTable<double>(20000, omega(2.5), omega(0.01), [dt](double x) { return cosh(x) * dt; });

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
      benchmark::DoNotOptimize(res = cosh(val) * dt);
    }
  }
  state.SetItemsProcessed(10000 * state.iterations());
}

// Register the function as a benchmark
BENCHMARK(BM_cosh_lookuptable);
BENCHMARK(BM_cosh);
// Run the benchmark
BENCHMARK_MAIN();
