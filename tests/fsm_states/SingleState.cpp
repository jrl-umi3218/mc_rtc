#include <mc_control/mc_fsm_state.h>

struct SingleState : public mc_control::FSMState
{
  void configure(const mc_rtc::Configuration &) override {}

  void start(mc_control::FSMController &) override {}

  bool run(mc_control::FSMController &) override { return false; }

  void teardown(mc_control::FSMController &) override {}
};

EXPORT_SINGLE_STATE("SingleState", SingleState, "OK")
