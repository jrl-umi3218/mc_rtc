#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/Robots.h>

#include <memory>
#include <nanobind/nanobind.h>
#include <nanobind/operators.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;
using namespace nb::literals;
using Robots = mc_rbdyn::Robots;
using RobotModule = mc_rbdyn::RobotModule;
using LoadRobotParameters = mc_rbdyn::LoadRobotParameters;

namespace mc_rtc_python
{

void bind_LoadRobotParameters(nb::module_ & m)
{
auto l = nb::class_<LoadRobotParameters>(m, "LoadRobotParameters");
l.def("base_tf", &LoadRobotParameters::base_tf,
"value"_a,
"Initial transformation betwen the base and the world")
                .def("base", &LoadRobotParameters::base,
                                                "Use this body as the base instead of the RobotModule provided one")
                .def("warn_on_missing_files",
                                                &LoadRobotParameters::warn_on_missing_files, "If true, print warning messages for missing files")
                .def("data", &LoadRobotParameters::data, "If provided, this is used as RobotData for the given robot, otherwise a new RobotData is created");
}

void bind_Robots(nb::module_ & m)
{
auto robots = nb::class_<mc_rbdyn::Robots>(m, "Robots");

// FIXME: symbol not found
// robots.def("robotModules",
//                                 &Robots::robotModules,
//                                 R"(
//   Give access to the underlying list of RobotModule objects
//                                 )");
                robots.def("mbs",
                                                nb::overload_cast<>(&Robots::mbs),
                                                "Give access to the underlying list of rbd::MultiBody objects")
                .def("mbs",
                                                nb::overload_cast<>(&Robots::mbs, nb::const_),
                                                "Give access to the underlying list of rbd::MultiBody objects (const)")
                .def("mbcs",
                                                nb::overload_cast<>(&Robots::mbs),
                                                "Give access to the underlying list of rbd::MultiBodyConfig objects")
                .def("mbcs",
                                                nb::overload_cast<>(&Robots::mbs, nb::const_),
                                                "Give access to the underlying list of rbd::MultiBodyConfig objects (const)");
robots.def("hasRobot", &Robots::hasRobot, "name"_a, "True if the given robot is part of this instance");


                robots.def("robotIndex", static_cast<unsigned int (Robots::*)() const>(&Robots::robotIndex), "Index of the main robot")
                .def("robotIndex", static_cast<unsigned int (Robots::*)(const std::string &) const>(&Robots::robotIndex), "name"_a,
                                                R"(
  Index of a robot by name

  :throws: if the robot does not exist
                                                )");
                robots.def("envIndex", &Robots::envIndex, "Index of the first non-actuated robot (or the last actuated robot if no unactuated robot are loaded)");

robots.def("load",
nb::overload_cast<const std::string &, const mc_rbdyn::RobotModule &, const mc_rbdyn::LoadRobotParameters &>(&Robots::load),
"name"_a, "module"_a, "params"_a,
R"(
   Load a single robot from a RobotModule with the provided parameters

  :param name: Name of the new robot. Must be unique.

  :param module: The RobotModule to fetch data from for this robot

  :param params: :py:class:LoadRobotParameters: for a description of the parameters

  :throws: If a robot named <name> already exists
)")
                .def("load",
nb::overload_cast<const mc_rbdyn::RobotModule &, const mc_rbdyn::LoadRobotParameters &>(&Robots::load),
"module"_a, "params"_a,
R"(
   Load a single robot from a RobotModule

  Use the name in the module to load the robot
)")
                .def("load",
nb::overload_cast<const std::vector<std::shared_ptr<mc_rbdyn::RobotModule>> &>(&Robots::load),
"modules"_a,
R"(
   Load multiple robots from as many RobotModule instances

  :param modules: List of RobotModule to load the robots from
)");

robots.def("robot", nb::overload_cast<>(&Robots::robot), nb::rv_policy::reference, "Access the main robot");
robots.def("robot", nb::overload_cast<>(&Robots::robot, nb::const_), nb::rv_policy::reference, "Access the main robot (const)");

m.def("loadRobot", nb::overload_cast<const RobotModule &, const LoadRobotParameters &>(&mc_rbdyn::loadRobot), nb::rv_policy::reference, "module"_a, "params"_a = LoadRobotParameters{});
m.def("loadRobot", nb::overload_cast<const std::string &, const RobotModule &, const LoadRobotParameters &>(&mc_rbdyn::loadRobot), nb::rv_policy::reference, "name"_a, "module"_a, "params"_a = LoadRobotParameters{});
}

}
