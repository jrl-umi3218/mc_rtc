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
using Configuration = mc_rtc::Configuration;

// specialization for configuration types
namespace mc_rtc
{
namespace internal
{
template<typename T>
std::string explicit_type_name()
{
  return "unknown";
}

template<>
std::string explicit_type_name<bool>()
{
  return "bool";
}

template<>
std::string explicit_type_name<int>()
{
  return "int";
}

template<>
std::string explicit_type_name<unsigned int>()
{
  return "uint";
}

template<>
std::string explicit_type_name<float>()
{
  return "float";
}

template<>
std::string explicit_type_name<double>()
{
  return "double";
}

template<>
std::string explicit_type_name<Eigen::Vector2d>()
{
  return "Vector2d";
}

template<>
std::string explicit_type_name<Eigen::Vector3d>()
{
  return "Vector3d";
}

template<>
std::string explicit_type_name<Eigen::Vector4d>()
{
  return "Vector4d";
}

template<>
std::string explicit_type_name<Eigen::Vector6d>()
{
  return "Vector6d";
}

template<>
std::string explicit_type_name<Eigen::VectorXd>()
{
  return "VectorXd";
}

template<>
std::string explicit_type_name<Eigen::Matrix3d>()
{
  return "Matrix3d";
}

template<>
std::string explicit_type_name<Eigen::Matrix6d>()
{
  return "Matrix6d";
}

template<>
std::string explicit_type_name<Eigen::MatrixXd>()
{
  return "MatrixXd";
}

template<>
std::string explicit_type_name<mc_rtc::Configuration>()
{
  return "Configuration";
}
} // namespace internal

template<typename T>
std::string explicit_type_name()
{
  return internal::explicit_type_name<std::decay_t<T>>();
}

} // namespace mc_rtc

namespace mc_rtc_python
{

template<typename T>
void f(T && parameter); // purposefully not defined

struct AddVisitor
{
  template<typename T, typename NBClass>
  static constexpr void Visit(NBClass & class_)
  {
    class_.def("add", static_cast<void (mc_rtc::Configuration::*)(const std::string &, T)>(&mc_rtc::Configuration::add),
               "key"_a, "value"_a,
               fmt::format("Add element of type {} to the configuration", mc_rtc::explicit_type_name<T>()).c_str());
  }
};

struct GetVisitor
{
  template<typename T, typename NBClass>
  static constexpr void Visit(NBClass & class_)
  {
    auto typeName = mc_rtc::explicit_type_name<T>();
    class_.def(("get_" + typeName).c_str(),
               [](Configuration & self, const std::string & key) { return static_cast<T>(self(key)); }, "key"_a,
               fmt::format("Get a value of type {} from the configuration", typeName).c_str());
    class_.def(
        ("get_" + typeName).c_str(),
        [](Configuration & self, const std::string & key, const T & default_value) -> std::decay_t<T>
        { return self.has(key) ? self(key) : default_value; }, "key"_a, "default_value"_a,
        fmt::format(
            "Get a value of type {} from the configuration if it exists. Otherwise return the provided default_value",
            typeName)
            .c_str());
  }
};

void bind_configuration(nb::module_ & m)
{
  auto c = nb::class_<mc_rtc::Configuration>(m, "Configuration");
  c.def(nb::init<>(), "Creates an empty configuration")
      .def(nb::init<const std::string &>(), "Create a configuration from file (yaml or json)")
      .def("has", &Configuration::has, "key"_a, "Check if the key is part of the configuration")
      .def("get_bool", [](Configuration & self, const std::string & key) { return static_cast<bool>(self(key)); })
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
      // .def("push", static_cast<void (Configuration::*)(const Eigen::Vector2d &)>(&Configuration::push))
      // TODO: missing eigen overloads

      .def("__getitem__", [](Configuration & self, const std::string & key) { return self(key); })
      // void operator()(const std::string & key, T & v) const
      .def("__call__", [](Configuration & self, const std::string & key) { return self(key); });

  // All overload types supported by Configuration
  // TODO: missing quaternion and sva types
  // clang-format off
  using ConfigurationTypes =
      mc_rtc::internal::TypeList<
      bool, double, int,
     const Eigen::Vector2d &, const Eigen::Vector3d &,
     const Eigen::Vector4d &, const Eigen::Vector6d &,
     const Eigen::VectorXd &,
     const Eigen::Matrix3d &, const Eigen::Matrix6d &,
     const Eigen::MatrixXd &,
     const Configuration &>;
  // clang-format on

  // Bind "add" for all supported types
  mc_rtc::internal::ForEach<ConfigurationTypes, AddVisitor>(c);
  // Bind "get_<type> for all supported types
  // This replaces T operator()(const std::string & key)
  // and           T operator()(const std::string & key, const T & default_value)
  //
  // Note: as python does not have conversion operators, if we wish to support the c++ idiom
  //    typed_assignement = config("key")
  // we would need to add a python factory to deduce it, something like
  //    int_val = config("key", int)
  // This is much more complicated and without real added value compared to get_<type>
  mc_rtc::internal::ForEach<ConfigurationTypes, GetVisitor>(c);
}

} // namespace mc_rtc_python
