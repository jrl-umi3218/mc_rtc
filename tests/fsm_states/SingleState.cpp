#include <mc_control/fsm/State.h>

struct SingleState : public mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration &) override {}

  void start(mc_control::fsm::Controller &) override {}

  bool run(mc_control::fsm::Controller &) override
  {
    return false;
  }

  void teardown(mc_control::fsm::Controller &) override {}
};

EXPORT_SINGLE_STATE("SingleState", SingleState)
