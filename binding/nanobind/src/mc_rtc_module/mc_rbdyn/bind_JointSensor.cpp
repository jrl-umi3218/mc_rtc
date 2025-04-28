#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

#include <mc_rbdyn/JointSensor.h>

namespace nb = nanobind;
using namespace nb::literals;
using JointSensor = mc_rbdyn::JointSensor;

namespace mc_rtc_python
{

void bind_JointSensor(nanobind::module_ & m)
{
  using JS = mc_rbdyn::JointSensor;
  auto jc = nb::class_<mc_rbdyn::JointSensor>(m, "JointSensor");
  jc.doc() = R"(
This structure defines a joint sensor that provides temperature and current information for a specfic joint.
)";

  jc.def(nb::init(), "Default constructor, does not represent a valid joint sensor")
      .def(nb::init<const std::string &>(), "jointName"_a, R"(
   :param jointName: Name of the joint to which the sensor is attached
  )");

  jc.def("joint", &JS::joint, ":returns: the sensor's joint name");

  jc.def_prop_rw(
      "motorTemperature", [](JS & self) { return self.motorTemperature(); }, [](JS & self, double t)
      { self.motorTemperature(t); }, "the sensor's motor temperature reading (Celcius), NaN if not provided");
  jc.def_prop_rw(
      "driverTemperature", [](JS & self) { return self.driverTemperature(); }, [](JS & self, double t)
      { self.driverTemperature(t); }, "the sensor's driver temperature reading (Celcius), NaN if not provided");
  jc.def_prop_rw(
      "motorCurrent", [](JS & self) { return self.motorCurrent(); }, [](JS & self, double t) { self.motorCurrent(t); },
      "the sensor's current reading (Ampere), NaN if not provided");
  jc.def_prop_rw(
      "motorStatus", [](JS & self) { return self.motorStatus(); }, [](JS & self, bool t) { self.motorStatus(t); },
      "the sensor's motor ON/OFF reading, ON (true) if not provided ");
}

} // namespace mc_rtc_python
