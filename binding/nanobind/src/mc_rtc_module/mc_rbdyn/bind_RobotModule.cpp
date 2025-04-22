#include <mc_rtc_python/mc_rbdyn/mc_rbdyn_module.h>
#include <mc_rbdyn/RobotModule.h>

#include <nanobind/stl/string.h>

namespace nb = nanobind;
using namespace nb::literals;
using RobotModule = mc_rbdyn::RobotModule;

namespace mc_rtc_python
{
  void bind_RobotModule(nanobind::module_ &m)
  {
      m.def("hello", []() { }, "hello documentation");
    auto c = nb::class_<mc_rbdyn::RobotModule>(m, "RobotModule");
    c.def(nb::init<const std::string &, const std::string &>(),
        R"(Construct from a provided path and name

As a result:
- name is defined as \p name
- path is defined as \p path
- urdf_path is path + /urdf/ + name + .urdf
- rsdf_dir is path + /rsdf/ + name
- calib_dir is path + /calib/ + name:q

No further action is taken. This constructor is useful to inherit from

Parameters:
- path Path to the robot description
- name Name of the robot)"),
    c.def(nb::init<const std::string &, const std::string &, const std::string &>(),
    R"(Construct from a provided path, name and urdf_path

  See: RobotModule(const std::string &, const std::string &)

  The different is that urdf_path is defined to \p urdf_path

  Parameters:
  - path Path to the robot description
  - name Name of the robot
  - urdf_path Path to the robot URDF)");

  // RobotModule(const std::string & name, const rbd::parsers::ParserResult & res);
  // void init(const rbd::parsers::ParserResult & res);



      // .def(nb::init<const std::string &>(), "Create a configuration from file (yaml or json)")
      // .def("has", &Configuration::has, "key"_a, "Check if the key is part of the configuration")
      // .def_static("rootArray", &Configuration::rootArray, "Return a Configuration with an array as root entry")
      // .def_static(
  }
}
