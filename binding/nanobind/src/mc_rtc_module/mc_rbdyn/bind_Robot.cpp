#include <mc_rbdyn/Robot.h>
// #include <RBDyn/MultiBodyConfig.h>

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
using Robot = mc_rbdyn::Robot;

namespace mc_rtc_python
{

void bind_Robot(nb::module_ & m)
{
  auto robot = nb::class_<mc_rbdyn::Robot>(m, "Robot");
  robot.def(
      "name", [](Robot & self) { return self.name(); },
      R"(
   Returns the name of the robot

   To rename a robot, use :py:func:rename:
                                )");
  //               .def("module", &Robot::module,
  //                                               "Retrieve the asociated RobotModule")
  //               .def("bodySensor", &Robot::bodySensor,
  //                                               ":returns: the first BodySensor in the robot")
  //               .def("addBodySensor", &Robot::addBodySensor,
  //                                               R"(
  //  Add BodySensor to the robot
  //
  // :param: sensor Body to add)")
  //               .def("hasBodySensor", &Robot::hasBodySensor,
  //                                               "name"_a,
  //                                               R"(
  //  :returns: true if the robot has a body sensor named name
  //
  // :param name: Name of the body sensor
  //                                               )")
  //               .def("bodyHasBodySensor", &Robot::bodyHasBodySensor,
  //                                               "body"_a,
  //                                               R"(
  // :returns: true if the specified body has a body sensor attached to it
  //
  // :param body: Body to query
  //                                               )");
  robot.def("mbc", [](Robot & self) -> rbd::MultiBodyConfig & { return self.mbc(); });
}

} // namespace mc_rtc_python
