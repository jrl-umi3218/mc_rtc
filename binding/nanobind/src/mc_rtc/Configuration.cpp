/**
 * This file adds python bindings for mc_rtc::Configuration
 *
 * Volontary omissions:
 * - Configuration::operator()(const std::string & key, T & v) : no mutable reference in python, use get_<type>(key)
 * instead.
 * - User defined conversions: how to handle them?
 */

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/operators.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

// Helper to call a visitor function for a list of types
// used here to specify bindings for all supported overloads of mc_rtc::Configuration
#include <state-observation/tools/definitions.hpp>
#include <typelist_visitor.h>

#include <mc_rtc/Configuration.h>
#include <mc_rtc/type_name.h>
#include <SpaceVecAlg/EigenTypedef.h>

namespace nb = nanobind;
using namespace nb::literals;
using Configuration = mc_rtc::Configuration;

template<typename T>
struct deduced_type; // purposefully not defined

template<typename T>
void show_deduced_type(T &&)
{
  deduced_type<T>::show;
}

template<typename T>
struct wrap_in_vector
{
  using type = std::vector<std::decay_t<T>> const &;
};

// specialization for configuration types
namespace mc_rtc
{
namespace internal
{

namespace traits
{
template<typename... Ts>
struct is_container : std::false_type
{
};

template<typename... Ts>
struct is_container<std::vector<Ts...>> : std::true_type
{
};

template<typename T>
inline constexpr auto is_container_v = is_container<T>::value;
} // namespace traits

template<typename T>
typename std::enable_if_t<std::is_same_v<bool, T>, std::string> explicit_type_name_()
{
  return "bool";
}

template<typename T>
typename std::enable_if_t<std::is_same_v<int, T>, std::string> explicit_type_name_()
{
  return "int";
}

template<typename T>
typename std::enable_if_t<std::is_same_v<unsigned int, T>, std::string> explicit_type_name_()
{
  return "uint";
}

template<typename T>
typename std::enable_if_t<std::is_same_v<float, T>, std::string> explicit_type_name_()
{
  return "float";
}

template<typename T>
typename std::enable_if_t<std::is_same_v<double, T>, std::string> explicit_type_name_()
{
  return "double";
}

template<typename T>
typename std::enable_if_t<std::is_same_v<Eigen::Vector2d, T>, std::string> explicit_type_name_()
{
  return "Vector2d";
}

template<typename T>
typename std::enable_if_t<std::is_same_v<Eigen::Vector3d, T>, std::string> explicit_type_name_()
{
  return "Vector3d";
}

template<typename T>
typename std::enable_if_t<std::is_same_v<Eigen::Vector4d, T>, std::string> explicit_type_name_()
{
  return "Vector4d";
}

template<typename T>
typename std::enable_if_t<std::is_same_v<Eigen::Vector6d, T>, std::string> explicit_type_name_()
{
  return "Vector6d";
}

template<typename T>
typename std::enable_if_t<std::is_same_v<Eigen::VectorXd, T>, std::string> explicit_type_name_()
{
  return "VectorXd";
}

template<typename T>
typename std::enable_if_t<std::is_same_v<Eigen::Matrix3d, T>, std::string> explicit_type_name_()
{
  return "Matrix3d";
}

template<typename T>
typename std::enable_if_t<std::is_same_v<Eigen::Matrix6d, T>, std::string> explicit_type_name_()
{
  return "Matrix6d";
}

template<typename T>
typename std::enable_if_t<std::is_same_v<Eigen::MatrixXd, T>, std::string> explicit_type_name_()
{
  return "MatrixXd";
}

template<typename T>
typename std::enable_if_t<std::is_same_v<std::string, T>, std::string> explicit_type_name_()
{
  return "str";
}

template<typename T>
typename std::enable_if_t<std::is_same_v<mc_rtc::Configuration, T>, std::string> explicit_type_name_()
{
  return "Configuration";
}

template<typename T>
typename std::enable_if_t<mc_rtc::internal::traits::is_container<std::decay_t<T>>::value, std::string>
    explicit_type_name_()
{
  using ValueT = typename std::decay_t<T>::value_type;
  // deduced_type<ValueT>::show;
  return "vector_" + explicit_type_name_<ValueT>();
}

} // namespace internal

template<typename T>
std::string explicit_type_name()
{
  return internal::explicit_type_name_<std::decay_t<T>>();
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
    // Overload resolution of the return type fails because we have both templated and non-template arguments
    // see https://github.com/pybind/pybind11/issues/1153 for details
    // static_cast<void (mc_rtc::Configuration::*)(const std::string &, T)>(&mc_rtc::Configuration::add),

    class_.def(
        "add", [](Configuration & self, const std::string & key, T value) { self.add(key, value); }, "key"_a, "value"_a,
        fmt::format("Add element of type {} to the configuration", mc_rtc::explicit_type_name<T>()).c_str());
  }
};

struct PushVisitor
{
  template<typename T, typename NBClass>
  static constexpr void Visit(NBClass & class_)
  {
    auto typeName = mc_rtc::explicit_type_name<T>();
    class_.def("push", static_cast<void (Configuration::*)(T)>(&Configuration::push), "value"_a,
               fmt::format(
                   R"(Insert a {0} element into an array

Parameters:
- value ({0}): Value to add

Throws: If the underlying Json value is not an array)",
                   typeName)
                   .c_str());
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
        { return self.has(key) ? static_cast<T>(self(key)) : default_value; }, "key"_a, "default_value"_a,
        fmt::format(
            "Get a value of type {} from the configuration if it exists. Otherwise return the provided default_value",
            typeName)
            .c_str());
  }
};

struct ArrayAtVisitor
{
  template<typename T, typename NBClass>
  static constexpr void Visit(NBClass & class_)
  {
    auto typeName = mc_rtc::explicit_type_name<T>();
    class_.def(
        "at", [](Configuration & self, size_t i, T v) { return self.at(i, v); }, "i"_a, "default_value"_a,
        fmt::format(R"(Retrieve a given value ({0}) from a JSON array

Parameters:
- i (size_t): Index to retrieve
- v ({0}): The default value

Returns:
the default value if the index is too high or if the underlying value does not match the requested type.)",
                    typeName)
            .c_str());
  }
};

// Binds
// template<typename T>
// T operator()(const std::string & key, const T & v) const
struct AssignmentOperatorVisitor
{
  template<typename T, typename NBClass>
  static constexpr void Visit(NBClass & class_)
  {
    auto typeName = mc_rtc::explicit_type_name<T>();
    class_.def(
        "__call__", [](Configuration & self, const std::string & key, T default_value)
        { return self(key, default_value); }, "key"_a, "default_value"_a,
        fmt::format(R"(Retrieve a given value (of type {0}) stored within the configuration with a default value.

If the key is not stored in the Configuration or if the underyling value
does not match the requested type, the default value is returned.

Parameters:
- key (str): The key used to store the value
- default_value ({0}): The default value)",
                    typeName)
            .c_str());
  }
};

// Bind operator ==
struct EqualityOperatorsVisitor
{
  template<typename T, typename NBClass>
  static constexpr void Visit(NBClass & class_)
  {
    class_.def("__eq__", [](const Configuration & lhs, T rhs) { return lhs == rhs; });
  }
};

void bind_configuration(nb::module_ & m)
{
  auto c = nb::class_<mc_rtc::Configuration>(m, "Configuration");
  c.def(nb::init<>(), "Creates an empty configuration")
      .def(nb::init<const std::string &>(), "Create a configuration from file (yaml or json)")
      .def("has", &Configuration::has, "key"_a, "Check if the key is part of the configuration")
      .def("rootArray", &Configuration::rootArray, "Return a Configuration with an array as root entry")
      .def_static(
          "fromJSONData", [](const std::string & data) { return Configuration::fromData(data); }, "json_data"_a,
          "Static constructor to load from JSON data")
      .def_static(
          "fromYAMLData", [](const std::string & data) { return Configuration::fromYAMLData(data); }, "json_data"_a,
          "Static constructor to load from YAML data")
      .def_static(
          "fromMessagePack",
          [](const std::string & data, size_t size) { return Configuration::fromMessagePack(data.c_str(), size); },
          "data"_a, "size"_a, "Static constructor to load from YAML data")
      .def("load", nb::overload_cast<const std::string &>(&Configuration::load), "path"_a,
           R"(Load additional data into the configuration

For any key existing in both objects:
- self(key) is overwritten for values and arrays
- config(key) is loaded into self(key) for objects
)")
      .def("load", nb::overload_cast<const mc_rtc::Configuration &>(&Configuration::load), "config"_a,
           R"(Load data from another Configuration object

For any key existing in both objects:
- self(key) is overwritten for values and arrays
- if config(key) and self(key) are objects, config(key) is loaded into self(key)
- otherwise, config(key) overwrites self(key)

Parameters:
config (str): The configuration object)")
      .def("save", &Configuration::save, "path"_a, "pretty"_a = true,
           R"(Save the configuration to a file.

If the path extension is yaml or yml then save in YAML format

Parameters:
path (str): Path to the configuration file

pretty (bool): Writes a human-readable file, defaults to true)")
      .def("dump", &Configuration::dump, "pretty"_a = false, "yaml"_a = false,
           R"(Dump the configuration into a string.

Parameters:
- pretty (bool): Writes a human-readable string, defaults to false
- yaml (bool): Writes YAML instead of JSON, defaults to false
)")
      .def("add_null", &Configuration::add_null, "key"_a,
           "Add a null element. Overrides the existing value if it holds one for the given key.")
      .def("array", nb::overload_cast<const std::string &, size_t>(&Configuration::array), "key"_a, "reserve"_a = 0,
           "Create an empty array")
      .def("array", nb::overload_cast<size_t>(&Configuration::array), "reserve"_a = 0, "Push an empty array")
      .def("object", &Configuration::object,
           R"(Push an empty object

See push(bool))")
      .def("empty", &Configuration::empty, "Return true if the underyling array is empty")
      .def("size", &Configuration::size, "If the stored value is an array return its size, otherwise returns 0")
      .def("isArray", &Configuration::isArray, "Returns true if the underlying element is an array")
      .def("isObject", &Configuration::isObject, "Returns true if the underlying value is an object")
      .def("isString", &Configuration::isString, "Returns true if the underlying value is a string")
      .def("isNumeric", &Configuration::isNumeric, "Returns true if the underlying value is numeric")
      // operator[]
      .def(
          "__getitem__", [](Configuration & self, size_t i) { return self[i]; }, "i"_a,
          R"(If the stored value is an array, returns a Configuration element for the i-th element.

Parameters:
- i (uint): Access i-th element

Throws: if i >= size())")
      .def("__getitem__", [](Configuration & self, const std::string & key) { return self(key); });

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
     const std::string &,
     const Configuration &>;
  // clang-format on

  using VectorConfigurationTypes = ConfigurationTypes::map<wrap_in_vector>;
  // deduced_type<VectorConfigurationTypes>::show;

  /**
   * Type List of all configuration types including container types (vector, etc)
   * TODO: type_cat template for variadic number of arguments
   **/
  using AllConfigurationTypes = mc_rtc::internal::type_cat<ConfigurationTypes, VectorConfigurationTypes>::type;

  // Bind "add" for all supported types
  mc_rtc::internal::ForEach<AllConfigurationTypes, AddVisitor>(c);
  // Bind "get_<type> for all supported types
  // This replaces T operator()(const std::string & key)
  // and           T operator()(const std::string & key, const T & default_value)
  //
  // Note: as python does not have conversion operators, if we wish to support the c++ idiom
  //    typed_assignement = config("key")
  // we would need to add a python factory to deduce it, something like
  //    int_val = config("key", int)
  // This is much more complicated and without real added value compared to get_<type>
  mc_rtc::internal::ForEach<AllConfigurationTypes, GetVisitor>(c);
  // Bind "push" for arrays
  mc_rtc::internal::ForEach<AllConfigurationTypes, PushVisitor>(c);
  // Bind "at(i, default_value)" for arrays
  mc_rtc::internal::ForEach<AllConfigurationTypes, ArrayAtVisitor>(c);
  // Binds
  // template<typename T>
  // T operator()(const std::string & key, const T & v) const
  mc_rtc::internal::ForEach<AllConfigurationTypes, AssignmentOperatorVisitor>(c);

  // Binds
  // template<typename t>
  // bool operator==(const t & rhs) const
  mc_rtc::internal::ForEach<AllConfigurationTypes, EqualityOperatorsVisitor>(c);
}

} // namespace mc_rtc_python
