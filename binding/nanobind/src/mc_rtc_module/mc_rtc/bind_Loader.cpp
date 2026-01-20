#include <nanobind/nanobind.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <mc_rtc/loader.h>

namespace mc_rtc_python
{

namespace nb = nanobind;
using namespace nb::literals;

void bind_Loader(nb::module_ & m)
{
  nb::exception<mc_rtc::LoaderException>(m, "LoaderException");

  nb::class_<mc_rtc::Loader>(m, "Loader",
                             "General wrapper for ltdl functionalities used to manage shared library loading.")

      // Static property for debug_suffix
      .def_prop_rw_static(
          "debug_suffix", [](nb::object /* self */) { return mc_rtc::Loader::debug_suffix; },
          [](nb::object /* self */, const std::string & suffix) { mc_rtc::Loader::debug_suffix = suffix; },
          "Suffix appended to library paths when running in debug mode (e.g., 'd').")

      .doc() = "The Loader class manages the underlying lt_dlhandle map and system initialization.";

}

} // namespace mc_rtc_python