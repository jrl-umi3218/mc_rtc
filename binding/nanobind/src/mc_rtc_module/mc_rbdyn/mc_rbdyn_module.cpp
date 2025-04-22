#include <mc_rtc_python/mc_rbdyn/mc_rbdyn_module.h>
#include <nanobind/nanobind.h>
#include <iostream>

namespace nb = nanobind;
using namespace nb::literals;

namespace mc_rtc_python
{

void bind_mc_rbdyn_module(nb::module_ & m)
{
  m.doc() = "mc_rbdyn bindings";
  m.def("test", []() { std::cout << "hello" << std::endl; }, "test doc");
  bind_RobotModule(m);
}

} // namespace mc_rtc_python
