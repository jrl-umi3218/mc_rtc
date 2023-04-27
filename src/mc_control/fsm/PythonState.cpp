/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/fsm/PythonState.h>

namespace mc_control
{

namespace fsm
{

void PythonState::update_python_failed()
{
  python_failed_ = handle_python_error();
}

void PythonState::configure(const mc_rtc::Configuration & config)
{
  if(!python_failed_)
  {
    configure_(config);
    update_python_failed();
  }
}

void PythonState::start(Controller & ctl)
{
  if(!python_failed_)
  {
    start_(ctl);
    update_python_failed();
  }
}

bool PythonState::run(Controller & ctl)
{
  if(!python_failed_)
  {
    bool ret = run_(ctl);
    update_python_failed();
    if(python_failed_) { return false; }
    return ret;
  }
  return false;
}

void PythonState::teardown(Controller & ctl)
{
  if(!python_failed_)
  {
    teardown_(ctl);
    update_python_failed();
  }
}

void PythonState::stop(Controller & ctl)
{
  if(!python_failed_)
  {
    stop_(ctl);
    update_python_failed();
  }
}

} // namespace fsm

} // namespace mc_control
