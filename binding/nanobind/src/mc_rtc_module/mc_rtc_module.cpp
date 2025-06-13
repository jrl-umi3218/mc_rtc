#include <mc_rtc_python/mc_control/mc_control_module.h>
#include <mc_rtc_python/mc_rbdyn/mc_rbdyn_module.h>
#include <mc_rtc_python/mc_rtc_module.h>
#include <nanobind/nanobind.h>

namespace nb = nanobind;
using namespace nb::literals;

NB_MODULE(_mc_rtc, _unsued_module)
{
  /**
   * We use a trick to load the symbols in the _sva namespace within the parent sva module's namespace.
   * This is done so that we expose the mc_rtc types as mc_rtc.Type instead of mc_rtc._mc_rtc.Type.
   *
   * In particular this makes types exported in stub files cleaner and more user-friendly, and makes sphinx autoapi
   * cross-references work accross projects.
   *
   * See https://github.com/wjakob/nanobind/issues/420#issuecomment-1950233151 for further details
   */
  (void)_unsued_module; // unused
  nb::module_ mc_rtc = nb::module_::import_("mc_rtc");

  // Import sva bindings
  nb::module_ sva = nb::module_::import_("sva");

  mc_rtc.doc() = "Python bindings for mc_rtc";

  nb::module_ mc_rtc_submodule = mc_rtc.def_submodule("mc_rtc", "mc_rtc submodule of 'mc_rtc'");
  mc_rtc_python::bind_configuration(mc_rtc_submodule);

  nb::module_ mc_rbdyn_submodule = mc_rtc.def_submodule("mc_rbdyn", "mc_rbdyn submodule of 'mc_rtc'");
  mc_rtc_python::bind_mc_rbdyn_module(mc_rbdyn_submodule);

  nb::module_ mc_control_submodule = mc_rtc.def_submodule("mc_control", "mc_control submodule of 'mc_rtc'");
  mc_rtc_python::bind_mc_control_module(mc_control_submodule);
}
