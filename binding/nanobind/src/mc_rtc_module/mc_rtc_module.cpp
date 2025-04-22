#include <mc_rtc_python/mc_rbdyn/mc_rbdyn_module.h>
#include <mc_rtc_python/mc_rtc_module.h>
#include <nanobind/nanobind.h>

namespace nb = nanobind;
using namespace nb::literals;

NB_MODULE(mc_rtc_python, mc_rtc_module)
{
  mc_rtc_module.doc() = "This is a \"hello world\" example with nanobind";

  mc_rtc_python::bind_configuration(mc_rtc_module);

  nb::module_ mc_rbdyn_module = mc_rtc_module.def_submodule("mc_rbdyn", "rbdyn submodule of 'mc_rtc'");
  mc_rtc_python::bind_mc_rbdyn_module(mc_rbdyn_module);
}
