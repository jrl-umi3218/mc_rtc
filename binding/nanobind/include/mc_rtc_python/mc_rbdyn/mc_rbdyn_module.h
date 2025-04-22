#pragma once
#include <nanobind/nanobind.h>

namespace mc_rtc_python
{
  void bind_mc_rbdyn_module(nanobind::module_ & m);
  void bind_RobotModule(nanobind::module_ & m);
}
