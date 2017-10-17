#include <mc_control/mc_fsm_state.h>

struct State1 : public mc_control::FSMState
{
  void configure(const mc_rtc::Configuration &) override {}

  void start(mc_control::FSMController &) override {}

  bool run(mc_control::FSMController &) override { return false; }

  void teardown(mc_control::FSMController &) override {}
};

struct State2 : public mc_control::FSMState
{
  void configure(const mc_rtc::Configuration &) override {}

  void start(mc_control::FSMController &) override {}

  bool run(mc_control::FSMController &) override { return false; }

  void teardown(mc_control::FSMController &) override {}
};

extern "C"
{
  FSM_STATE_API std::vector<std::string> MC_RTC_FSM_STATE()
  {
    return {"State1", "State2"};
  }

  FSM_STATE_API void destroy(mc_control::FSMState * ptr) { delete ptr; }

  FSM_STATE_API mc_control::FSMState * create(const std::string & n)
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

  FSM_STATE_API std::vector<std::string> outputs(const std::string & n)
  {
    if(n == "State1")
    {
      return {"OK"};
    }
    if(n == "State2")
    {
      return {"OK", "NOK", "FAILURE"};
    }
    return {};
  }
}
