#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/states/Parallel.h>

namespace mc_control
{

namespace fsm
{

void ParallelState::configure(const mc_rtc::Configuration & config)
{
  config_.load(config);
}

void ParallelState::start(Controller & ctl)
{
  std::vector<std::string> states = config_("states");
  if(states.size() == 0)
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "ParallelState requires at least one state to run")
  }
  auto states_config = config_("configs", mc_rtc::Configuration{});
  states_.reserve(states.size());
  for(auto & s : states)
  {
    states_.emplace_back(ctl.factory().create(s, ctl, states_config(s, mc_rtc::Configuration{})));
  }
}

bool ParallelState::run(Controller & ctl)
{
  bool ret = true;
  for(auto & s : states_)
  {
    ret = s->run(ctl) && ret;
  }
  if(ret)
  {
    output(states_.back()->output());
  }
  return ret;
}

void ParallelState::stop(Controller & ctl)
{
  for(auto & s : states_)
  {
    s->stop(ctl);
  }
}

void ParallelState::teardown(Controller & ctl)
{
  for(auto & s : states_)
  {
    s->teardown_(ctl);
  }
}

bool ParallelState::read_msg(std::string & msg)
{
  for(auto & s : states_)
  {
    if(s->read_msg(msg))
    {
      return true;
    }
  }
  return false;
}

bool ParallelState::read_write_msg(std::string & msg, std::string & out)
{
  for(auto & s : states_)
  {
    if(s->read_write_msg(msg, out))
    {
      return true;
    }
  }
  return false;
}

} // namespace fsm

} // namespace mc_control

EXPORT_SINGLE_STATE("Parallel", mc_control::fsm::ParallelState)
