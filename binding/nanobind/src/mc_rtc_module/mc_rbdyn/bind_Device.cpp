#include <mc_rbdyn/BodySensor.h>
#include <mc_rbdyn/Device.h>
#include <mc_rbdyn/Robot.h>
#include <SpaceVecAlg/SpaceVecAlg>

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/unordered_map.h>
#include <nanobind/stl/vector.h>
#include <nanobind/trampoline.h>

namespace nb = nanobind;
using namespace nb::literals;
using Device = mc_rbdyn::Device;
using DevicePtr = mc_rbdyn::DevicePtr;

namespace mc_rtc_python
{

/**
 * Trampoline class to handle virtual functions in python
 * See: https://nanobind.readthedocs.io/en/latest/classes.html#trampolines
 **/
struct PyDevice : Device
{
  NB_TRAMPOLINE(Device, 1);

  DevicePtr clone() const override { NB_OVERRIDE_PURE(clone); }
};

void bind_Device(nb::module_ & m)
{
  auto rd = nb::class_<Device, PyDevice>(m, "Device");
  auto bs = nb::class_<mc_rbdyn::BodySensor, Device>(m, "BodySensor");

  rd.doc() = R"(
This class represents a generic device attached to a robot.

This is a barebone interface meant to be derived by a concrete device implementation.
)";

  rd.def(nb::init<const std::string &>(), "name"_a, R"(
   Build a device not specifically attached to the robot

  When the Device becomes part of a Robot instance it is attached to the
  origin of the robot)")
      .def(nb::init<const std::string &, const std::string &, const sva::PTransformd &>(), "name"_a, "parent"_a,
           "X_p_s"_a, "Build a device attached to a specific body of the robot");

  rd.def("name", &Device::name, ":returns: the name of the sensor")
      .def("type", &Device::type, ":returns: the type of the sensor")
      .def_prop_rw(
          "parent", [](Device & self) -> const std::string & { return self.parent(); },
          [](Device & self, const std::string & p) { self.parent(p); }, "Get or change the parent body of the sensor")
      .def_prop_rw(
          "X_p_d", [](Device & self) -> const sva::PTransformd & { return self.X_p_d(); },
          [](Device & self, const sva::PTransformd & pt) { self.X_p_d(pt); },
          "Transformation from the parent body to the device")
      .def_prop_rw(
          "X_p_s", [](Device & self) -> const sva::PTransformd & { return self.X_p_s(); },
          [](Device & self, const sva::PTransformd & pt) { self.X_p_s(pt); },
          "Transformation from the parent body to the sensor")
      .def("X_0_d", &Device::X_0_d, "robot"_a,
           "Returns the device position in the inertial frame (convenience function)")
      .def("X_0_s", &Device::X_0_s, "robot"_a,
           "Returns the sensor position in the inertial frame (convenience function)");
}

} // namespace mc_rtc_python
