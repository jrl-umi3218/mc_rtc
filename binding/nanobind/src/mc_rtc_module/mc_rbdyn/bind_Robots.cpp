#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/RobotModule.h>
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
using RobotsPtr = mc_rbdyn::RobotsPtr;
using Robot = mc_rbdyn::Robot;
using RobotModule = mc_rbdyn::RobotModule;
using LoadRobotParameters = mc_rbdyn::LoadRobotParameters;

namespace mc_rtc_python
{

void bind_LoadRobotParameters(nb::module_ & m)
{
  auto l = nb::class_<mc_rbdyn::LoadRobotParameters>(m, "LoadRobotParameters");
  l.def(nb::init());
  l.def("base_tf", &LoadRobotParameters::base_tf, "value"_a, "Initial transformation betwen the base and the world")
      .def("base", &LoadRobotParameters::base, "Use this body as the base instead of the RobotModule provided one")
      .def("warn_on_missing_files", &LoadRobotParameters::warn_on_missing_files,
           "If true, print warning messages for missing files")
      .def("data", &LoadRobotParameters::data,
           "If provided, this is used as RobotData for the given robot, otherwise a new RobotData is created");
}

void bind_Robots(nb::module_ & m)
{
  auto robots = nb::class_<mc_rbdyn::Robots>(m, "Robots");

  robots.def("__init__",
             [](mc_rbdyn::Robots * /* self */, nb::args args, nb::kwargs kwargs)
             {
               // This is a special pattern in nanobind/pybind11:
               // We cannot easily modify 'self' in-place for shared_ptr holders in __init__.
               // Instead, we usually bind a static method like __new__ OR use a factory that returns the instance.

               // HOWEVER, since 'Robots' is held by shared_ptr, binding __init__ directly is tricky
               // because nanobind has already allocated the shell.

               // The most robust fix for shared_ptr factory classes is usually static factories,
               // BUT to keep python syntax `Robots()`, we use __new__.
             });

  robots.def("__new__",
             [](nb::type_object type, nb::args args, nb::kwargs kwargs)
             {
               bool skip_alloc = false;
               if(kwargs.contains("skip_alloc")) { skip_alloc = nb::cast<bool>(kwargs["skip_alloc"]); }

               // Case 1: No arguments -> Robots()
               if(args.size() == 0)
               {
                 if(skip_alloc)
                 {
                   // Return a "null" shared_ptr logic if needed,
                   // or just an empty container.
                   // Note: Returning nullptr here might crash Python if it expects an object.
                   // It is safer to return an empty shell or throw if skip_alloc isn't supported safely.
                   return std::shared_ptr<mc_rbdyn::Robots>(nullptr);
                 }
                 return mc_rbdyn::Robots::make();
               }

               // Case 2: Copy constructor -> Robots(other)
               if(args.size() == 1)
               {
                 if(nb::isinstance<mc_rbdyn::Robots>(args[0]))
                 {
                   auto other = nb::cast<std::shared_ptr<mc_rbdyn::Robots>>(args[0]);
                   auto new_robots = mc_rbdyn::Robots::make();
                   if(other) { other->copy(*new_robots); }
                   return new_robots;
                 }
               }

               throw nb::type_error("Robots(): incompatible constructor arguments.");
             });

  // FIXME: symbol not found
  // robots.def("robotModules",
  //                                 &Robots::robotModules,
  //                                 R"(
  //   Give access to the underlying list of RobotModule objects
  //                                 )");
  robots.def("mbs", nb::overload_cast<>(&Robots::mbs), "Give access to the underlying list of rbd::MultiBody objects")
      .def("mbs", nb::overload_cast<>(&Robots::mbs, nb::const_),
           "Give access to the underlying list of rbd::MultiBody objects (const)")
      .def("mbcs", nb::overload_cast<>(&Robots::mbs),
           "Give access to the underlying list of rbd::MultiBodyConfig objects")
      .def("mbcs", nb::overload_cast<>(&Robots::mbs, nb::const_),
           "Give access to the underlying list of rbd::MultiBodyConfig objects (const)");
  robots.def("hasRobot", &Robots::hasRobot, "name"_a, "True if the given robot is part of this instance");

  robots
      .def("robotIndex", static_cast<unsigned int (Robots::*)() const>(&Robots::robotIndex), "Index of the main robot")
      .def("robotIndex", static_cast<unsigned int (Robots::*)(const std::string &) const>(&Robots::robotIndex),
           "name"_a,
           R"(
  Index of a robot by name

  :throws: if the robot does not exist
                                                )");
  robots.def("envIndex", &Robots::envIndex,
             "Index of the first non-actuated robot (or the last actuated robot if no unactuated robot are loaded)");

  robots
      .def("load",
           nb::overload_cast<const std::string &, const mc_rbdyn::RobotModule &, const mc_rbdyn::LoadRobotParameters &>(
               &Robots::load),
           "name"_a, "module"_a, "params"_a = mc_rbdyn::LoadRobotParameters(), nb::rv_policy::take_ownership, 
           R"(
   Load a single robot from a RobotModule with the provided parameters

  :param name: Name of the new robot. Must be unique.

  :param module: The RobotModule to fetch data from for this robot

  :param params: :py:class:LoadRobotParameters: for a description of the parameters

  :throws: If a robot named <name> already exists
)")
      .def("load",
           nb::overload_cast<const mc_rbdyn::RobotModule &, const mc_rbdyn::LoadRobotParameters &>(&Robots::load),
           "module"_a, "params"_a = mc_rbdyn::LoadRobotParameters(), nb::rv_policy::take_ownership, 
           R"(
   Load a single robot from a RobotModule

  Use the name in the module to load the robot
)")
      .def("load", nb::overload_cast<const std::vector<std::shared_ptr<mc_rbdyn::RobotModule>> &>(&Robots::load),
           "modules"_a, nb::rv_policy::take_ownership, 
           R"(
   Load multiple robots from as many RobotModule instances

  :param modules: List of RobotModule to load the robots from
)");

  robots.def("robot", static_cast<Robot & (Robots::*)()>(&Robots::robot), nb::rv_policy::reference,
             ":returns: The main robot");

  // MC_RBDYN_DLLAPI RobotsPtr loadRobots(const std::vector<std::shared_ptr<RobotModule>> & modules);
  m.def("loadRobots", &mc_rbdyn::loadRobots, "modules"_a);

  // FIXME: causes std::bad_cast, maybe because RobotDataPtr is not bound?
  // { return mc_rbdyn::loadRobot(rm, params); });
  // m.def("loadRobot", static_cast<RobotsPtr (*)(const RobotModule &, const LoadRobotParameters
  // &)>(&mc_rbdyn::loadRobot), "module"_a, "params"_a = LoadRobotParameters{});
  m.def(
      "loadRobot", [](const mc_rbdyn::RobotModule & rm) -> RobotsPtr { return mc_rbdyn::loadRobot(rm); }, "module"_a,
      "Load robot from a RobotModule. The name of the robot is that of :py:arg:`RobotModule.name`");

  // m.def("loadRobot", nb::overload_cast<const std::string &, const RobotModule &, const LoadRobotParameters
  // &>(&mc_rbdyn::loadRobot), nb::rv_policy::reference, "name"_a, "module"_a, "params"_a = LoadRobotParameters{});
}

} // namespace mc_rtc_python
