#include <mc_control/fsm/State.h>

struct State1 : public mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration &) override {}

  void start(mc_control::fsm::Controller &) override {}

  bool run(mc_control::fsm::Controller &) override
  {
    return false;
  }

  void teardown(mc_control::fsm::Controller &) override {}
};

struct State2 : public mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration &) override {}

  void start(mc_control::fsm::Controller &) override {}

  bool run(mc_control::fsm::Controller &) override
  {
    return false;
  }

  void teardown(mc_control::fsm::Controller &) override {}
};

extern "C"
{
  FSM_STATE_API void MC_RTC_FSM_STATE(std::vector<std::string> & names)
  {
    names = {"State1", "State2"};
  }

  FSM_STATE_API void destroy(mc_control::fsm::State * ptr)
  {
    delete ptr;
  }

  FSM_STATE_API mc_control::fsm::State * create(const std::string & n)
  {
    if(n == "State1")
    {
      return new State1();
    }
    if(n == "State2")
    {
      return new State2();
    }
    return nullptr;
  }
}
