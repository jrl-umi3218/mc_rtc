#include <mc_control/fsm/PythonState.h>

namespace mc_control
{

namespace fsm
{

void PythonState::configure(const mc_rtc::Configuration & config)
{
  configure_(config);
}

void PythonState::start(Controller & ctl)
{
  start_(ctl);
}

bool PythonState::run(Controller & ctl)
{
  return run_(ctl);
}

void PythonState::teardown(Controller & ctl)
{
  teardown_(ctl);
}

void PythonState::stop(Controller & ctl)
{
  stop_(ctl);
}

} // namespace fsm

} // namespace mc_control
