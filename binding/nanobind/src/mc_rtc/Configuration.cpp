#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/eigen/dense.h>

// Helper to call a visitor function for a list of types
// used here to specify bindings for all supported overloads of mc_rtc::Configuration
#include <typelist_visitor.h>

#include <mc_rtc/Configuration.h>

namespace nb = nanobind;
using namespace nb::literals;

namespace mc_rtc_python
{
    // XXX: Replace with lamda
    struct CallVisitor {
        template <typename T> static constexpr void Visit(nb::module_ &m)
        {
            std::cout << "visit for type " << mc_rtc::type_name<T>() << std::endl;
            m.def("__call__",
                    [](mc_rtc::Configuration &self, const std::string &key, T value)
                    {
                    self(key, value);
                    }
                 );

        }
    };


    void bind_configuration(nb::module_ & m)
    {
        using Configuration = mc_rtc::Configuration;
        nb::class_<mc_rtc::Configuration>(m, "Configuration")
            .def(nb::init<>(), "Creates an empty configuration")
            .def(nb::init<const std::string &>(), "Create a configuration from file (yaml or json)")
            .def("add_null", &Configuration::add_null, "key"_a, "Add a null element. Overrides the existing value if it holds one for the given key.")
            .def("array",
                    nb::overload_cast<const std::string &, size_t>(&Configuration::array),
                    "key"_a,
                    "reserve"_a = 0,
                    "Create an empty array")
            .def("array",
                    nb::overload_cast<size_t>(&Configuration::array),
                    "reserve"_a = 0,
                    "Push an empty array")
            // Overload resolution of the return type fails because we have both templated and non-template arguments
            // see https://github.com/pybind/pybind11/issues/1153 for details
            .def("push", static_cast<void (Configuration::*)(bool)>(&Configuration::push))
            // .def("push", static_cast<void (Configuration::*)(int8_t)>(&Configuration::push))
            // .def("push", static_cast<void (Configuration::*)(uint8_t)>(&Configuration::push))
            // .def("push", static_cast<void (Configuration::*)(int16_t)>(&Configuration::push))
            // .def("push", static_cast<void (Configuration::*)(uint16_t)>(&Configuration::push))
            // .def("push", static_cast<void (Configuration::*)(int32_t)>(&Configuration::push))
            // .def("push", static_cast<void (Configuration::*)(uint32_t)>(&Configuration::push))
            // .def("push", static_cast<void (Configuration::*)(int64_t)>(&Configuration::push))
            // .def("push", static_cast<void (Configuration::*)(uint64_t)>(&Configuration::push))
            .def("push", static_cast<void (Configuration::*)(double)>(&Configuration::push))
            .def("push", static_cast<void (Configuration::*)(const std::string &)>(&Configuration::push))
            // .def("push", static_cast<void (Configuration::*)(const char *)>(&Configuration::push))
            .def("push", static_cast<void (Configuration::*)(const Eigen::Vector2d &)>(&Configuration::push))
            // TODO: missing eigen overloads

            .def("__getitem__",
                    [](Configuration &self, const std::string &key)
                    {
                    return self(key);
                    }
                )
            // void operator()(const std::string & key, T & v) const
            .def("__call__",
                    [](Configuration &self, const std::string &key, double value)
                    {
                    self(key, value);
                    }
                );

        // All overload types supported by Configuration
        using ConfigurationTypes = mc_rtc::internal::TypeList<bool, int8_t, uint8_t, int16_t, uint16_t, int32_t, uint32_t, int64_t, uint64_t, double>;

        // For all supported types,
        // bind void operator()(const std::string & key, T & v) const
        using Visitor = CallVisitor;
        // XXX: Allow to pass an arbitrary lambda to the Visitor
        // mc_rtc::internal::ForEach<ConfigurationTypes, Visitor>(m);
    }

}
