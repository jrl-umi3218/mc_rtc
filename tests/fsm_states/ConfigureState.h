#pragma once

#include <mc_control/mc_fsm_state.h>

struct MC_CONTROL_DLLAPI ConfigureState : mc_control::FSMState
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::FSMController&) override {}

  bool run(mc_control::FSMController&) override { return false; }

  void teardown(mc_control::FSMController&) override {}

  unsigned int value() const { return value_; }
protected:
  unsigned int value_ = 0;
};
