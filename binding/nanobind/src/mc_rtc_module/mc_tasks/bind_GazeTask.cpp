#include <mc_tasks/GazeTask.h>
#include <mc_rbdyn/Robots.h>

#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/eigen/dense.h>

namespace nb = nanobind;
using namespace nb::literals;

namespace mc_rtc_python
{

void bind_GazeTask(nb::module_ & m)
{
  using mc_tasks::GazeTask;

  nb::class_<GazeTask>(m, "GazeTask",
      R"(Control the gaze of a robot body or camera frame.

This task allows controlling the gaze towards a 2D point in the image plane or a 3D point in space.
It is backend-aware:
- Tasks backend: uses tasks::qp::GazeTask
- TVM backend: uses details::TVMGazeTask)")

  .def(nb::init<const mc_rbdyn::RobotFrame &,
                  double,
                  double,
                  const Eigen::Vector3d &>(),
         "frame"_a,
         "stiffness"_a = 2.0,
         "weight"_a = 500.0,
         "error"_a = Eigen::Vector3d::UnitZ(),
         R"(Create a GazeTask from a control frame.

Parameters
----------
frame : mc_rbdyn.RobotFrame
stiffness : float
weight : float
error : Eigen::Vector3d, optional initial error vector
)")

    .def(nb::init<const std::string &,
                  const Eigen::Vector2d &,
                  double,
                  const sva::PTransformd &,
                  const mc_rbdyn::Robots &,
                  unsigned int,
                  double,
                  double>(),
         "bodyName"_a,
         "point2d"_a,
         "depthEstimate"_a,
         "X_b_gaze"_a,
         "robots"_a,
         "robotIndex"_a,
         "stiffness"_a = 2.0,
         "weight"_a = 500.0,
         R"(Create a GazeTask targeting a 2D point with depth estimate.)")

    .def(nb::init<const std::string &,
                  const Eigen::Vector3d &,
                  const sva::PTransformd &,
                  const mc_rbdyn::Robots &,
                  unsigned int,
                  double,
                  double>(),
         "bodyName"_a,
         "point3d"_a,
         "X_b_gaze"_a,
         "robots"_a,
         "robotIndex"_a,
         "stiffness"_a = 2.0,
         "weight"_a = 500.0,
         R"(Create a GazeTask targeting a 3D point.)")

    .def("reset", &GazeTask::reset,
         R"(Reset the task target to the current body orientation.)")

    .def("error",
         nb::overload_cast<const Eigen::Vector2d &, const Eigen::Vector2d &>(&GazeTask::error),
         "point2d"_a,
         "point2d_ref"_a = Eigen::Vector2d::Zero(),
         R"(Set the current 2D error in the camera frame.)")

    .def("error",
         nb::overload_cast<const Eigen::Vector3d &, const Eigen::Vector2d &>(&GazeTask::error),
         "point3d"_a,
         "point2d_ref"_a = Eigen::Vector2d::Zero(),
         R"(Set the current 3D error in the camera frame.)");
}

} // namespace mc_rtc_python
