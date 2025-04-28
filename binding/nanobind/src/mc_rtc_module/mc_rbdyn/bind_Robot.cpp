#include <mc_rbdyn/ForceSensor.h>
#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/RobotModule.h>
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
using RobotModule = mc_rbdyn::RobotModule;
using BodySensor = mc_rbdyn::BodySensor;
using ForceSensor = mc_rbdyn::ForceSensor;
using JointSensor = mc_rbdyn::JointSensor;
using Device = mc_rbdyn::Device;

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
  robot
      .def("module", static_cast<const RobotModule & (Robot::*)() const>(&Robot::module),
           "Retrieve the asociated RobotModule")
      .def("bodySensor", static_cast<const BodySensor & (Robot::*)() const>(&Robot::bodySensor),
           ":returns: the first BodySensor in the robot")
      .def("bodySensor", static_cast<const BodySensor & (Robot::*)(const std::string &) const>(&Robot::bodySensor),
           "name"_a,
           R"(
  :returns: a specific BobySensor by name

  :param name: Name of the sensor

  :throws: If the sensor does not exist
                    )")
      .def("addBodySensor", &Robot::addBodySensor,
           R"(
   Add BodySensor to the robot

  :param: sensor Body to add)")
      .def("hasBodySensor", &Robot::hasBodySensor, "name"_a,
           R"(
   :returns: true if the robot has a body sensor named name

  :param name: Name of the body sensor
                                                )")
      .def("bodyHasBodySensor", &Robot::bodyHasBodySensor, "body"_a,
           R"(
  :returns: true if the specified body has a body sensor attached to it

  :param body: Body to query
                                                )")
      .def("bodyBodySensor",
           static_cast<const BodySensor & (Robot::*)(const std::string &) const>(&Robot::bodyBodySensor), "name"_a,
           R"(
  :returns: a specific BodySensor by body name

  :param name: Name of the body

  :throws: If there is no sensor attached to the body
                    )");

  robot
      .def("jointHasJointSensor", &Robot::jointHasJointSensor, "joint"_a,
           R"(
  :returns: true if the specified joint has a joint sensor attached to it

  :param joint: Joint to query
                                                )")
      .def("jointJointSensor",
           static_cast<const JointSensor & (Robot::*)(const std::string &) const>(&Robot::jointJointSensor), "name"_a,
           R"(
  :returns: a specific JointSensor by joint name

  :param name: Name of the joint

  :throws: If there is no sensor attached to the joint
                    )")
      .def("jointSensors", static_cast<const std::vector<JointSensor> & (Robot::*)() const>(&Robot::jointSensors),
           ":returns: all joint sensors");

  robot.def("hasJoint", &Robot::hasJoint, "name"_a, ":returns: True if the robot has a joint named **name**")
      .def("hasBody", &Robot::hasBody, "name"_a, ":returns: True if the robot has a body named **name**")
      .def("hasFrame", &Robot::hasFrame, "name"_a, ":returns: True if the robot has a frame named **name**");

  robot.def(
      "devices",
      [](Robot & self) -> std::vector<Device *>
      { // converts from DevicePtrVector
        auto & rd = self.devices();
        std::vector<Device *> devices(rd.size());
        std::transform(rd.begin(), rd.end(), devices.begin(), [](const auto & device) { return device.get(); });
        std::cout << "r devices size: " << rd.size() << std::endl;
        std::cout << "devices size: " << devices.size() << std::endl;
        return devices;
      },
      "Get all devices attached to a robot");

  robot.def("mbc", [](Robot & self) -> rbd::MultiBodyConfig & { return self.mbc(); });
}

} // namespace mc_rtc_python
