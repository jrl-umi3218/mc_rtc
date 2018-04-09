#include <mc_control/fsm/Executor.h>

#include <mc_control/fsm/Controller.h>

namespace mc_control
{

namespace fsm
{

void Executor::init(Controller & ctl, const mc_rtc::Configuration & config)
{
  config_ = config;
  config("Managed", managed_);
  config("StepByStep", step_by_step_);
  if(!managed_)
  {
    transition_map_.init(ctl.factory(), config);
    transition_triggered_ = true;
    next_state_ = transition_map_.initState();
  }
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

  //FIXME ready is used to avoid a potential "yoyo" effect here, i.e. if a
  //state estimated it was completed once, the future estimations don't matter
  if(state_)
  {
    if(! (state_->run(ctl) || ready_) )
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
    auto trans = transition_map_.transition(curr_state_,state_output_);
    if(!trans.first) // No more transition, execution complete
    {
      complete_ = true;
      return complete(ctl, keep_state);
    }
    next_state_ = trans.second.state;
    auto type = trans.second.type;
    if(type == Transition::Type::Auto ||
       (type == Transition::Type::StepByStep && !step_by_step_) ||
       transition_triggered_)
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
    state_->teardown(ctl);
    state_ = nullptr;
  }
}

bool Executor::complete(Controller & ctl, bool keep_state)
{
  if(!keep_state)
  {
    state_->teardown(ctl);
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
  if(!ready_ || next_state_.empty()) { return; }
  ready_ = false;
  transition_triggered_ = false;
  LOG_SUCCESS("Startig state " << next_state_)
  if(state_) { state_->teardown(ctl); }
  if(config_.has("configs") && config_("configs").has(next_state_))
  {
    state_ = ctl.factory().create(next_state_, ctl, config_("configs")(next_state_));
  }
  else
  {
    state_ = ctl.factory().create(next_state_, ctl);
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
