#include <mc_rbdyn/Device.h>
#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/RobotData.h>

#include <nanobind/nanobind.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/unordered_map.h>
#include <nanobind/stl/vector.h>
#include <unordered_map>

namespace nb = nanobind;
using namespace nb::literals;
using RobotData = mc_rbdyn::RobotData;

namespace mc_rtc_python
{

// XXX: check that using bound attributes such as grippers/gripperRef/devices
// does not break/have unexpected behaviour. Do they need to be bound in the first place?
// Or should we let MCController's internal handle them
void bind_RobotData(nb::module_ & m)
{
  // using RD = RobotData;
  // auto rd = nb::class_<RobotData>(m, "RobotData");
  //
  // rd.doc() = R"(
  // Hold data and objects that are shared among different views of a robot (control/real/output)
  //
  // The framework enforce consistency between these views
  //
  // This includes:
  //
  // * reference joint order
  // * encoder level readings (position/velocity/torques)
  // * force/body/joint sensors
  // * grippers
  // * devices
  // )";
  //
  // rd.def(nb::init());
  // rd.def_rw("refJointOrder", &RD::refJointOrder, "Reference joint order, see :py:class:`RobotModule`")
  //     .def_rw("encderValues", &RD::encoderValues, "Encoders' positions provided in the robot's ref joint order")
  //     .def_rw("encderVelocities", &RD::encoderVelocities,
  //             R"(Encoders' velocities provided in the robot's ref joint order)")
  //     .def_rw("jointTorques", &RD::jointTorques, "Joint torques provided by the low-level controller")
  //     .def_rw("forceSensors", &RD::forceSensors, "Force sensors")
  //     .def_rw("forceSensorIndex", &RD::forceSensorsIndex,
  //             R"(Correspondance between force sensor's name and force sensor index)")
  //     .def_rw("bodyForceSensors", &RD::bodyForceSensors_,
  //             R"(Correspondance between bodies' names and attached force sensors)")
  //     .def_rw("bodySensors", &RD::bodySensors, "Hold all body sensors")
  //     .def_rw("bodySensorsIndex", &RD::bodySensorsIndex,
  //             R"(Correspondance between body sensor's name and body sensor index)")
  //     .def_rw("bodyBodySensors", &RD::bodyBodySensors,
  //             R"(Correspondance between bodies' names and attached body sensors)")
  //     .def_rw("jointSensors", &RD::jointSensors, R"(Hold all joint sensors)")
  //     .def_rw("jointJointSensors", &RD::jointJointSensors,
  //             R"(Correspondance between joints' names and attached joint sensors)");
  //
  // // XXX: will that work as expected?
  // rd.def_prop_rw(
  //     "grippers",
  //     [](RD & self)
  //     {
  //       std::unordered_map<std::string, mc_control::Gripper *> gout;
  //       for(const auto & [name, gripper] : self.grippers) { gout.emplace(std::make_pair(name, gripper.get())); }
  //       return gout;
  //     },
  //     [](RD & self, const std::unordered_map<std::string, mc_control::Gripper *> & grippers)
  //     {
  //       self.grippers.clear();
  //       for(const auto & [name, gripper] : grippers) { self.grippers.emplace(std::make_pair(name, gripper)); }
  //     });
  //
  // rd.def("grippersRef",
  //             [](RD & self)
  //             { // convert as vector of pointers to remove use of reference_wrapper
  //               const auto & gin = self.grippersRef;
  //               std::vector<mc_control::Gripper *> gout(gin.size());
  //               std::transform(gin.begin(), gin.end(), gout.begin(),
  //                       [](const auto & g) { return &g.get(); });
  //               return gout;
  //             },
  //             R"(:returns: all grippers)");
  //
  // // XXX: will this work as expected?
  // rd.def("devices",
  //        [](RD & self)
  //        {
  //          std::vector<mc_rbdyn::Device *> out;
  //          for(const auto & device : self.devices) { out.emplace_back(device.get()); }
  //          return out;
  //        })
  //     .def("devices",
  //          [](RD & self, const std::vector<mc_rbdyn::Device *> & devices)
  //          {
  //            self.devices.clear();
  //            for(const auto & device : devices) { self.devices.emplace_back(device); }
  //          });
  //
  // rd.def_rw("devicesIndex", &RD::devicesIndex, R"(Correspondance between a device's name and a device index)");
  //
  // rd.def_rw("robots", &RD::robots, R"(A list of robots that share this data
  //
  //   This is used to communicate changes to the data to all instances that share this data)");
}

} // namespace mc_rtc_python
