#include <mc_rtc_python/mc_rtc_module.h>
#include <nanobind/nanobind.h>

namespace nb = nanobind;
using namespace nb::literals;

NB_MODULE(_mc_rtc, m)
{
  m.doc() = "This is a \"hello world\" example with nanobind";
  m.def("add", [](int a, int b, int c) { return a + b; }, "a"_a, "b"_a, "c"_a);

  mc_rtc_python::bind_configuration(m);
}
