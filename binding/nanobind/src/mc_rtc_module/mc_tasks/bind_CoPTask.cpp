#include <mc_rbdyn/Robots.h>
#include <mc_tasks/CoPTask.h>

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;
using namespace nb::literals;

namespace mc_rtc_python
{

void bind_CoPTask(nb::module_ & m)
{
  using mc_tasks::force::CoPTask;
  nb::class_<CoPTask>(m, "CoPTask",
                      R"(Track Center-of-Pressure (CoP) references at a contact surface.

This task extends AdmittanceTask by tracking CoP instead of torques.
When surface pressure is zero, torque tracking is automatically disabled.)")

      .def(nb::init<const mc_rbdyn::RobotFrame &, double, double>(), "frame"_a, "stiffness"_a = 5.0,
           "weight"_a = 1000.0, R"(Create a CoPTask from a frame which must have a force sensor attached.)");

     //  .def(nb::init<const std::string &, const mc_rbdyn::Robots &, unsigned, double, double>(), "robotSurface"_a,
     //       "robots"_a, "robotIndex"_a, "stiffness"_a = 5.0, "weight"_a = 1000.0,
     //       R"(Create a CoPTask from a surface name for the selected robot.)")

     //  .def("reset", &CoPTask::reset,
     //       R"(Reset task: set target position to current, reset targets to zero, disable CoP tracking.)")

     //  .def_prop_rw(
     //      "useTargetPressure", [](const CoPTask & cop) { return cop.useTargetPressure(); },
     //      [](CoPTask & cop, bool s) { cop.useTargetPressure(s); },
     //      R"(Whether desired torque uses target pressure (true) or measured pressure (false).)")

     //  .def("measuredCoP", &CoPTask::measuredCoP, R"(Measured Center-of-Pressure (2D) in the control frame.)")

     //  .def("measuredCoPW", &CoPTask::measuredCoPW, R"(Measured Center-of-Pressure (3D) in world frame.)")

     //  .def("setZeroTargetWrench", &CoPTask::setZeroTargetWrench, R"(Reset target wrench and target CoP to zero.)")

     //  .def("targetCoP", nb::overload_cast<>(&CoPTask::targetCoP, nb::const_), nb::rv_policy::reference_internal,
     //       R"(Return target CoP in the control frame.)")

     //  .def("targetCoP", nb::overload_cast<const Eigen::Vector2d &>(&CoPTask::targetCoP), "targetCoP"_a,
     //       R"(Set target CoP in the control frame.)")

     //  .def("targetCoPW", &CoPTask::targetCoPW, R"(Return target CoP in world coordinates.)")

     //  .def("targetForce", nb::overload_cast<>(&CoPTask::targetForce, nb::const_), nb::rv_policy::reference_internal,
     //       R"(Return target force in the control frame.)")

     //  .def("targetForce", nb::overload_cast<const Eigen::Vector3d &>(&CoPTask::targetForce), "force"_a,
     //       R"(Set target force in the control frame.)")

     //  .def("targetForceW", &CoPTask::targetForceW, "forceW"_a, R"(Set target force using world-frame coordinates.)")

     //  .def("targetWrench", static_cast<const sva::ForceVecd & (CoPTask::*)() const>(&CoPTask::targetWrench),
     //       nb::rv_policy::reference_internal, "Return target wrench in the control frame.")

     //  .def("load", &CoPTask::load, "solver"_a, "config"_a, R"(Load CoP task parameters from a configuration.)");
}

} // namespace mc_rtc_python
