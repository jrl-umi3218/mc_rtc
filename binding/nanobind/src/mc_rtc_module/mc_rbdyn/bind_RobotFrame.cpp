#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/RobotFrame.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;
using namespace nb::literals;
using namespace mc_rbdyn;

namespace mc_rtc_python
{

void bind_RobotFrame(nb::module_ & m)
{
  auto rf = nb::class_<mc_rbdyn::RobotFrame>(m, "RobotFrame");
  rf.doc() = R"(
A frame that belongs to a Robot instance

The frame quantities are deduced from the robot's state

Every RobotFrame has either an explicit parent RobotFrame or an implicit parent (the robot's body it is attached to).
Hence setting its position via the \ref Frame interface only sets relative position and settings its velocity through
the same interface never has any effect.)";

  rf.def(
        "robot_const", [](RobotFrame & self) -> const Robot & { return self.robot(); }, nb::rv_policy::reference,
        "The robot to which this frame is attached (const)")
      .def(
          "robot", [](RobotFrame & self) -> Robot & { return self.robot(); }, nb::rv_policy::reference,
          "The robot to which this frame is attached (const)");

  rf.def("bodyMbcIndex", &RobotFrame::bodyMbcIndex, "Returns this frame body index in robot's mbc")
      .def("body", &RobotFrame::body, "The body to which this frame is attached to")
      .def("position", &RobotFrame::position, "Computes the frame position")
      .def("velocity", &RobotFrame::velocity, "Computes the frame velocity")
      .def("X_b_f", &RobotFrame::X_b_f, "Compute the transformation from the body to this frame")
      .def_prop_rw(
          "X_p_f", [](RobotFrame & self) -> const sva::PTransformd & { return self.X_p_f(); },
          [](RobotFrame & self, sva::PTransformd pt) { self.X_p_f(pt); }, "position"_a,
          "Transformation from the parent's frame/body to the frame")
      .def("hasForceSensor", &RobotFrame::hasForceSensor,
           "True if the frame has a foce sensor (direct or indirect) attached")
      .def("forceSensor", &RobotFrame::forceSensor, nb::rv_policy::reference,
           R"(
   Returns the force sensor attached to the frame (const)

  :throws: if \ref hasForceSensor() is False)")
      .def("wrench", &RobotFrame::wrench, R"(
   Returns the force sensor gravity-free wrench in this frame

  :throws: if \ref hasForceSensor() is false
      )")
      .def("cop", &RobotFrame::cop, "min_pressure"_a = 0.5,
           R"(
   Compute the CoP in frame coordinates from gravity free measurements

  :param min_pressure: Minimum pressure in N (default: 0.5N)

  :returns: Measured CoP in frame coordinates if pressure >= min_pressure, zero otherwise

  :throws: if :py:func:`hasForceSensor` is False
  )")
      .def("copW", &RobotFrame::copW, "min_pressure"_a = 0.5,
           R"(
   Compute the CoP in inertial frame from gravity free measurements

  :param min_pressure: Minimum pressure in N (default: 0.5N)

  :returns: Measured CoP in inertial frame if pressure >= min_pressure, zero otherwise

  :throws: if :py:func:`hasForceSensor` is False
  )")
      .def("makeFrame", &RobotFrame::makeFrame, "name"_a, "X_p_f"_a, "baked"_a = false,
           R"(
   Create a frame whose parent is this frame

  :param name: Name of the new frame
  :param X_p_f: Transformation from this frame to the newly created frame
  :param baked: Attach the newly created frame to the parent body of this frame rather than the frame
  :returns: The new frame
              )");
}

} // namespace mc_rtc_python
