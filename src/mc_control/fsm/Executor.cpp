#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/Executor.h>

namespace mc_control
{

namespace fsm
{

void Executor::init(Controller & ctl,
                    const mc_rtc::Configuration & config,
                    const std::string & name,
                    const std::vector<std::string> & category)
{
  config_ = config;
  name_ = name;
  config("Managed", managed_);
  config("StepByStep", step_by_step_);
  if(!managed_)
  {
    transition_map_.init(ctl.factory(), config);
    transition_triggered_ = true;
    next_state_ = transition_map_.initState();
  }
  auto gui = ctl.gui();
  if(gui)
  {
    if(category.size())
    {
      category_ = category;
    }
    else
    {
      category_ = {"FSM"};
      if(name.size())
      {
        category_.push_back(name);
      }
    }
    gui->addElement(category_, mc_rtc::gui::Button("Interrupt", [this]() { interrupt(); }),
                    mc_rtc::gui::Label("Current state", [this]() { return state(); }),
                    mc_rtc::gui::Label("Next state ready", [this]() { return ready(); }),
                    mc_rtc::gui::Label("Next state", [this]() { return next_state(); }),
                    mc_rtc::gui::Button("Start next state", [this]() { next(); }),
                    mc_rtc::gui::Form("Force transition",
                                      [this](const mc_rtc::Configuration & c) { this->resume(c("State")); },
                                      mc_rtc::gui::FormDataComboInput("State", true, {"states"})));
  }
  std::string log_entry = "Executor";
  if(name_.size())
  {
    log_entry += "_" + name_;
  }
  else
  {
    log_entry += "_Main";
  }
  ctl.logger().addLogEntry(log_entry, [this]() { return curr_state_; });
}

bool Executor::run(Controller & ctl, bool keep_state)
{
  if(interrupt_triggered_)
  {
    interrupt_triggered_ = false;
    if(state_)
    {
      state_->stop(ctl);
      LOG_WARNING("Interrupted " << curr_state_)
      complete(ctl, false);
    }
  }

  // FIXME ready is used to avoid a potential "yoyo" effect here, i.e. if a
  // state estimated it was completed once, the future estimations don't matter
  if(state_)
  {
    if(!(state_->run(ctl) || ready_))
    {
      return false;
    }
    if(!ready_)
    {
      ready_ = true;
      state_output_ = state_->output();
      LOG_SUCCESS("Completed " << curr_state_)
    }
    if(managed_)
    {
      complete_ = true;
      return complete(ctl, keep_state);
    }
    auto trans = transition_map_.transition(curr_state_, state_output_);
    if(!trans.first) // No more transition, execution complete
    {
      complete_ = true;
      return complete(ctl, keep_state);
    }
    next_state_ = trans.second.state;
    auto type = trans.second.type;
    if(type == Transition::Type::Auto || (type == Transition::Type::StepByStep && !step_by_step_)
       || transition_triggered_)
    {
      next(ctl);
      return true;
    }
    return complete(ctl, keep_state);
  }
  else if(transition_triggered_)
  {
    next(ctl);
  }
  return ready_;
}

void Executor::stop(Controller & ctl)
{
  if(state_)
  {
    state_->stop(ctl);
  }
}

void Executor::teardown(Controller & ctl)
{
  if(state_)
  {
    state_->teardown_(ctl);
    state_ = nullptr;
  }
  if(ctl.gui())
  {
    ctl.gui()->removeCategory(category_);
  }
  std::string log_entry = "Executor";
  if(name_.size())
  {
    log_entry += "_" + name_;
  }
  else
  {
    log_entry += "_Main";
  }
  ctl.logger().removeLogEntry(log_entry);
}

bool Executor::complete(Controller & ctl, bool keep_state)
{
  if(!keep_state)
  {
    state_->teardown_(ctl);
    state_ = nullptr;
  }
  ready_ = true;
  return true;
}

bool Executor::next()
{
  if(!managed_ && ready_)
  {
    transition_triggered_ = true;
    return true;
  }
  return false;
}

void Executor::next(Controller & ctl)
{
  if(!ready_ || next_state_.empty())
  {
    return;
  }
  ready_ = false;
  transition_triggered_ = false;
  LOG_SUCCESS("Starting state " << next_state_)
  if(state_)
  {
    state_->teardown_(ctl);
    ctl.resetPostures();
  }
  if(config_.has("configs") && config_("configs").has(next_state_))
  {
    state_ = ctl.factory().create(next_state_, ctl, config_("configs")(next_state_));
  }
  else
  {
    state_ = ctl.factory().create(next_state_, ctl);
  }
  auto gui = ctl.gui();
  if(gui)
  {
    for(const auto & s : transition_map_.transitions(curr_state_))
    {
      gui->removeElement(category_, "Force transition to " + s);
    }
    for(const auto & s : transition_map_.transitions(next_state_))
    {
      gui->addElement(category_, mc_rtc::gui::Button("Force transition to " + s, [this, s]() { this->resume(s); }));
    }
  }
  curr_state_ = next_state_;
  next_state_ = "";
}

bool Executor::resume(const std::string & state)
{
  if(!interrupt_triggered_ && state_ && !ready_)
  {
    LOG_WARNING(curr_state_ << " interrupted to  resume " << state)
  }
  complete_ = false;
  interrupt_triggered_ = true;
  transition_triggered_ = true;
  next_state_ = state;
  return true;
}

bool Executor::read_msg(std::string & msg)
{
  return state_ && state_->read_msg(msg);
}

bool Executor::read_write_msg(std::string & msg, std::string & out)
{
  return state_ && state_->read_write_msg(msg, out);
}

} // namespace fsm

} // namespace mc_control
