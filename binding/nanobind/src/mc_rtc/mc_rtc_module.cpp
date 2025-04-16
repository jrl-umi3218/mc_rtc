#include <mc_rtc_python/mc_rtc_module.h>
#include <nanobind/nanobind.h>

namespace nb = nanobind;
using namespace nb::literals;

NB_MODULE(mc_rtc_python, m)
{
  m.doc() = "This is a \"hello world\" example with nanobind";

  mc_rtc_python::bind_configuration(m);
}
