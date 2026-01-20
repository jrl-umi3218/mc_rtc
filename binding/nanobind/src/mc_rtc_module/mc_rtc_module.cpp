#include <mc_rtc_python/mc_control/mc_control_module.h>
#include <mc_rtc_python/mc_rbdyn/mc_rbdyn_module.h>
#include <mc_rtc_python/mc_tasks/mc_tasks_module.h> 
#include <mc_rtc_python/mc_solver/mc_solver_module.h> 
#include <mc_rtc_python/mc_rtc_module.h>
#include <nanobind/nanobind.h>

namespace nb = nanobind;
using namespace nb::literals;

NB_MODULE(_mc_rtc, m)
{
  // Import dependent bindings
  nb::module_ sva = nb::module_::import_("sva");
  nb::module_ rbdyn = nb::module_::import_("rbdyn");

  m.doc() = "Python bindings for mc_rtc";

  nb::module_ mc_rtc_submodule = m.def_submodule("mc_rtc", "mc_rtc submodule of 'mc_rtc'");
  mc_rtc_python::bind_configuration(mc_rtc_submodule);
  mc_rtc_python::bind_Loader(mc_rtc_submodule);

  nb::module_ mc_rbdyn_submodule = m.def_submodule("mc_rbdyn", "mc_rbdyn submodule of 'mc_rtc'");
  mc_rtc_python::bind_mc_rbdyn_module(mc_rbdyn_submodule);

  nb::module_ mc_control_submodule = m.def_submodule("mc_control", "mc_control submodule of 'mc_rtc'");
  mc_rtc_python::bind_mc_control_module(mc_control_submodule);

  nb::module_ mc_tasks_submodule = m.def_submodule("mc_tasks", "mc_tasks submodule of 'mc_rtc'");
  mc_rtc_python::bind_mc_tasks_module(mc_tasks_submodule);

  nb::module_ mc_solver_submodule = m.def_submodule("mc_solver", "mc_solver submodule of 'mc_rtc'");
  mc_rtc_python::bind_mc_solver_module(mc_solver_submodule);
}
