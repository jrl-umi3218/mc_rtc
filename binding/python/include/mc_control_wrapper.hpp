/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/mc_python_controller.h>

#include <Eigen/Core>
#include <functional>
#include <memory>
#include <sstream>

using array7d = std::array<double, 7>;

namespace mc_control
{

ControllerResetData & const_cast_crd(const ControllerResetData & in)
{
  return const_cast<ControllerResetData &>(in);
}

typedef bool (*run_callback_t)(void *);
typedef void (*reset_callback_t)(const ControllerResetData &, void *);

void set_run_callback(MCPythonController & ctl, run_callback_t fn, void * data)
{
  ctl.run_callback = std::bind(fn, data);
}

void set_reset_callback(MCPythonController & ctl, reset_callback_t fn, void * data)
{
  ctl.reset_callback = std::bind(fn, std::placeholders::_1, data);
}

} // namespace mc_control
