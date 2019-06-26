/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_rtc/Configuration.h>

#include <functional>

using configure_cb = std::function<void(const mc_rtc::Configuration &)>;
using controller_cb = std::function<void(mc_control::fsm::Controller &)>;
using run_cb = std::function<bool(mc_control::fsm::Controller &)>;

template<typename T, typename U>
configure_cb make_configure_cb(T python_trampoline, U cb)
{
  return [python_trampoline, cb](const mc_rtc::Configuration & conf) {
    return python_trampoline(cb, const_cast<mc_rtc::Configuration &>(conf));
  };
}

template<typename T, typename U>
controller_cb make_controller_cb(T python_trampoline, U cb)
{
  return [python_trampoline, cb](mc_control::fsm::Controller & ctl) { return python_trampoline(cb, ctl); };
};

template<typename T, typename U>
run_cb make_run_cb(T python_trampoline, U cb)
{
  return [python_trampoline, cb](mc_control::fsm::Controller & ctl) { return python_trampoline(cb, ctl); };
};
