/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/states/Meta.h>

namespace mc_control
{

namespace fsm
{

void MetaState::start(Controller & ctl)
{
  if(!config_.has("StepByStep"))
  {
    config_.add("StepByStep", false);
  }
  executor_.init(ctl, config_, name(), config_("category", std::vector<std::string>{}));
  run(ctl);
}

bool MetaState::run(Controller & ctl)
{
  executor_.run(ctl, true);
  bool ret = executor_.complete();
  if(ret)
  {
    output(executor_.output());
  }
  return ret;
}

void MetaState::stop(Controller & ctl)
{
  executor_.stop(ctl);
}

void MetaState::teardown(Controller & ctl)
{
  executor_.teardown(ctl);
}

bool MetaState::read_msg(std::string & msg)
{
  std::stringstream ss;
  std::string name_;
  ss << msg;
  ss >> name_;
  if(name_ == name())
  {
    std::string token;
    ss >> token;
    if(token == "interrupt")
    {
      executor_.interrupt();
      return true;
    }
    if(token == "play")
    {
      std::string state;
      ss >> state;
      return executor_.resume(state);
    }
    if(token == "next")
    {
      return executor_.next();
    }
  }
  return false;
}

std::vector<std::vector<std::string>> MetaState::transitions() const
{
  return config_("transitions");
}

std::map<std::string, mc_rtc::Configuration> MetaState::configs() const
{
  return config_("configs", mc_rtc::Configuration{});
}

} // namespace fsm

} // namespace mc_control

EXPORT_SINGLE_STATE("Meta", mc_control::fsm::MetaState)
