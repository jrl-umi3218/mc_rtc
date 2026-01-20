#include <mc_rtc_python/mc_rbdyn/mc_rbdyn_module.h>
#include <nanobind/nanobind.h>

namespace nb = nanobind;
using namespace nb::literals;

namespace mc_rtc_python
{

void bind_mc_rbdyn_module(nb::module_ & m)
{
  m.doc() = "mc_rbdyn bindings";
  bind_Device(m);
  bind_ForceSensorCalibData(m);
  bind_ForceSensor(m);
  bind_JointSensor(m);
  bind_RobotModule(m);
  bind_RobotData(m);
  bind_RobotFrame(m);
  bind_LoadRobotParameters(m);
  bind_Robot(m);
  bind_Robots(m);
}

} // namespace mc_rtc_python
