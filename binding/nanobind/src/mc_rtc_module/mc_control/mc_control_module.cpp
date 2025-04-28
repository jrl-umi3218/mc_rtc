#include <iostream>
#include <mc_rtc_python/mc_control/mc_control_module.h>
#include <nanobind/nanobind.h>

namespace nb = nanobind;
using namespace nb::literals;

namespace mc_rtc_python
{

void bind_mc_control_module(nb::module_ & m)
{
  nb::module_ sva = nb::module_::import_("sva");
  m.doc() = "mc_control bindings";
  bind_Gripper(m);
}

} // namespace mc_rtc_python
