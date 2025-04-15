#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

// Helper to call a visitor function for a list of types
// used here to specify bindings for all supported overloads of mc_rtc::Configuration
#include <typelist_visitor.h>

#include <mc_rtc/Configuration.h>
#include <mc_rtc/type_name.h>

namespace nb = nanobind;
using namespace nb::literals;

namespace mc_rtc_python
{
// XXX: Replace with lamda
struct AddVisitor
{
  template<typename T>
  static constexpr void Visit(nb::module_ & m)
  {
    std::cout << "visit for type " << mc_rtc::type_name<T>() << std::endl;
    m.def("add", static_cast<void (mc_rtc::Configuration::*)(const std::string &, double)>(&mc_rtc::Configuration::add),
          "key"_a, "value"_a,
          fmt::format("Add an object of type {} to the configuration", mc_rtc::type_name<T>()).c_str());
  }
};

void bind_configuration(nb::module_ & m)
{
  using Configuration = mc_rtc::Configuration;
  auto c = nb::class_<mc_rtc::Configuration>(m, "Configuration");
  c.def(nb::init<>(), "Creates an empty configuration")
      .def(nb::init<const std::string &>(), "Create a configuration from file (yaml or json)")
      .def("add_null", &Configuration::add_null, "key"_a,
           "Add a null element. Overrides the existing value if it holds one for the given key.")
      .def("array", nb::overload_cast<const std::string &, size_t>(&Configuration::array), "key"_a, "reserve"_a = 0,
           "Create an empty array")
      .def("array", nb::overload_cast<size_t>(&Configuration::array), "reserve"_a = 0, "Push an empty array")
      // Overload resolution of the return type fails because we have both templated and non-template arguments
      // see https://github.com/pybind/pybind11/issues/1153 for details
      .def("push", static_cast<void (Configuration::*)(bool)>(&Configuration::push))
      .def("push", static_cast<void (Configuration::*)(int64_t)>(&Configuration::push))
      .def("push", static_cast<void (Configuration::*)(double)>(&Configuration::push))
      .def("push", static_cast<void (Configuration::*)(const std::string &)>(&Configuration::push))
      .def("push", static_cast<void (Configuration::*)(const Eigen::Vector2d &)>(&Configuration::push))
      // TODO: missing eigen overloads

      .def("__getitem__", [](Configuration & self, const std::string & key) { return self(key); })
      // void operator()(const std::string & key, T & v) const
      .def("__call__", [](Configuration & self, const std::string & key) { return self(key); })
      .def("__call__", [](Configuration & self, const std::string & key, double & value) { self(key, value); });

  // All overload types supported by Configuration
  using ConfigurationTypes = mc_rtc::internal::TypeList<bool, double>;
  // For all supported types,
  // XXX: Allow to pass an arbitrary lambda to the Visitor
  // mc_rtc::internal::ForEach<ConfigurationTypes, AddVisitor>(m);

  c.def("add", static_cast<void (Configuration::*)(const std::string &, double)>(&Configuration::add), "key"_a,
        "value"_a, fmt::format("Add an object of type {} to the configuration", mc_rtc::type_name<double>()).c_str());
}

} // namespace mc_rtc_python
