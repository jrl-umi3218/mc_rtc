#include <iostream>
#include <mc_rtc_python/mc_rbdyn/mc_rbdyn_module.h>
#include <nanobind/nanobind.h>

namespace nb = nanobind;
using namespace nb::literals;

namespace mc_rtc_python
{

void bind_mc_rbdyn_module(nb::module_ & m)
{
  nb::module_ sva = nb::module_::import_("sva");
  m.doc() = "mc_rbdyn bindings";
  bind_ForceSensorCalibData(m);
  bind_ForceSensor(m);
  bind_RobotModule(m);
  bind_Robots(m);
  bind_Robot(m);
}

} // namespace mc_rtc_python
