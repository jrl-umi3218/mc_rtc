#pragma once
#include <nanobind/nanobind.h>

namespace mc_rtc_python
{
void bind_configuration(nanobind::module_ & m);
void bind_Loader(nanobind::module_ & m);
}
