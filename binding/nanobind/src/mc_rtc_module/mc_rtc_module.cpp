#include <mc_rtc_python/mc_rbdyn/mc_rbdyn_module.h>
#include <mc_rtc_python/mc_rtc_module.h>
#include <nanobind/nanobind.h>

namespace nb = nanobind;
using namespace nb::literals;

NB_MODULE(mc_rtc_python, mc_rtc_module)
{
  nb::module_ sva = nb::module_::import_("sva");

  mc_rtc_module.doc() = "This is a \"hello world\" example with nanobind";

  nb::module_ mc_rtc_submodule = mc_rtc_module.def_submodule("mc_rtc", "mc_rtc submodule of 'mc_rtc'");
  mc_rtc_python::bind_configuration(mc_rtc_submodule);

  nb::module_ mc_rbdyn_submodule = mc_rtc_module.def_submodule("mc_rbdyn", "mc_rbdyn submodule of 'mc_rtc'");
  mc_rtc_python::bind_mc_rbdyn_module(mc_rbdyn_submodule);
}
