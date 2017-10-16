#include <mc_control/fsm_states/Pause.h>

#include <mc_control/mc_fsm_controller.h>

namespace mc_control
{

void PauseState::configure(const mc_rtc::Configuration & config)
{
  config("duration", duration_);
}

void PauseState::start(FSMController & ctl)
{
  assert(duration_ >= 0);
  tick_ = 0;
  goal_ = std::ceil(duration_ / ctl.solver().dt());
}

bool PauseState::run(FSMController&)
{
  if(++tick_ > goal_)
  {
    output("OK");
    return true;
  }
  return false;
}

}

EXPORT_SINGLE_STATE("Pause", mc_control::PauseState, "OK")
