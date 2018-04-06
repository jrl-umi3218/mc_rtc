#pragma once

#include <mc_control/fsm/states/Meta.h>

#include <mc_control/fsm/Controller.h>

namespace mc_control
{

namespace fsm
{

void MetaState::configure(const mc_rtc::Configuration & config)
{
  config_.load(config);
}

void MetaState::start(Controller & ctl)
{
  if(!config_.has("StepByStep"))
  {
    config_.add("StepByStep", false);
  }
  bool sbs = config_("StepByStep");
  auto gui = ctl.gui();
  if(gui)
  {
    gui->addElement({"FSM"},
                    mc_rtc::gui::Label(name() + "- current state",
                                       [this](){ return executor_.state(); }));
    if(sbs)
    {
      gui->addElement({"FSM"},
                      mc_rtc::gui::Label(name() + "- next state ready",
                                         [this](){ return executor_.ready(); }),
                      mc_rtc::gui::Button(name() + "- next state",
                                          [this](){ executor_.next(); }));
    }
  }
  executor_.init(ctl, config_);
}

bool MetaState::run(Controller & ctl)
{
  executor_.run(ctl, true);
  bool ret = !executor_.running();
  if(ret)
  {
    output(executor_.output());
  }
  return ret;
}

void MetaState::teardown(Controller & ctl)
{
  bool sbs = config_("StepByStep");
  auto gui = ctl.gui();
  if(gui)
  {
    gui->removeElement({"FSM"}, name() + "- current state");
    if(sbs)
    {
      gui->removeElement({"FSM"}, name() + "- next state ready");
      gui->removeElement({"FSM"}, name() + "- next state");
    }
  }
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

} // namespace fsm

} // namespace mc_control

EXPORT_SINGLE_STATE("Meta", mc_control::fsm::MetaState, "OK")
