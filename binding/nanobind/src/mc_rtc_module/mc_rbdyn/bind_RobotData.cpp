#include <mc_rbdyn/RobotData.h>

#include <nanobind/nanobind.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/unordered_map.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;
using namespace nb::literals;
using RobotData = mc_rbdyn::RobotData;

namespace mc_rtc_python
{

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
  // rd.def_rw("refJointOrder", &RD::refJointOrder, "Reference joint order, see :py:class:`RobotModule`");
  // rd.def_rw("encderValues", &RD::encoderValues, "Encoders' positions provided in the robot's ref joint order");
  // rd.def_rw("encderVelocities", &RD::encoderVelocities, "Encoders' velocities provided in the robot's ref joint
  // order"); rd.def_rw("jointTorques", &RD::jointTorques, "Joint torques provided by the low-level controller");
}

} // namespace mc_rtc_python
