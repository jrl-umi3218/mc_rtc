#include <mc_tasks/OrientationTask.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;
using namespace nb::literals;

namespace mc_rtc_python
{

void bind_OrientationTask(nb::module_ & m)
{
  using mc_tasks::OrientationTask;

  nb::class_<OrientationTask, mc_tasks::TrajectoryTaskGeneric>(m, "OrientationTask")
      .def(nb::init<const mc_rbdyn::RobotFrame &, double, double>(), "frame"_a, "stiffness"_a = 2.0, "weight"_a = 500.0)

      .def(nb::init<const std::string &, const mc_rbdyn::Robots &, unsigned int, double, double>(), "bodyName"_a,
           "robots"_a, "robotIndex"_a, "stiffness"_a = 2.0, "weight"_a = 500.0)

      .def("reset", &OrientationTask::reset)

      // Orientation Getter/Setter
      .def("orientation", nb::overload_cast<const Eigen::Matrix3d &>(&OrientationTask::orientation), "ori"_a,
           "Set the frame orientation target")
      .def("orientation", nb::overload_cast<>(&OrientationTask::orientation),
           "Get the current frame orientation target");
}

} // namespace mc_rtc_python