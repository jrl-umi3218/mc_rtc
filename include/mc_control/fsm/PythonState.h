#pragma once

#include <mc_control/fsm/State.h>

namespace mc_control
{

namespace fsm
{

/** Provides an interface for the Python bindings to fill-out */
struct MC_CONTROL_FSM_DLLAPI PythonState : public State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(Controller & ctl) override;

  bool run(Controller & ctl) override;

  void teardown(Controller & ctl) override;

  void stop(Controller & ctl) override;

  std::function<void(const mc_rtc::Configuration &)> configure_ = [](const mc_rtc::Configuration &) {};
  std::function<void(Controller &)> start_ = [](Controller &) {};
  std::function<bool(Controller &)> run_ = [](Controller &) { return false; };
  std::function<void(Controller &)> teardown_ = [](Controller &) {};
  std::function<void(Controller &)> stop_ = [](Controller &) {};
};

} // namespace fsm

} // namespace mc_control
