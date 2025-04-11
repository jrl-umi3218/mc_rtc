#include <nanobind/nanobind.h>
#include <mc_rtc_python/mc_rtc_module.h>

namespace nb = nanobind;
using namespace nb::literals;

NB_MODULE(mc_rtc_python, m) {
    m.doc() = "This is a \"hello world\" example with nanobind";
    m.def("add", [](int a, int b, int c) { return a + b; }, "a"_a, "b"_a, "c"_a);

    mc_rtc_python::bind_configuration(m);
}
