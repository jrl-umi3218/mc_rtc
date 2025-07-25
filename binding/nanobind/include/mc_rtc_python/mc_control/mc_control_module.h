#pragma once
#include <nanobind/nanobind.h>

namespace mc_rtc_python
{

void bind_mc_control_module(nanobind::module_ & m);

void bind_Gripper(nanobind::module_ & m);

} // namespace mc_rtc_python
