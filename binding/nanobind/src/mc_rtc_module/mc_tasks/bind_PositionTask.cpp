#include <mc_tasks/PositionTask.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;
using namespace nb::literals;

namespace mc_rtc_python
{

void bind_PositionTask(nb::module_ & m)
{
  using mc_tasks::PositionTask;

  nb::class_<PositionTask, mc_tasks::TrajectoryTaskGeneric>(m, "PositionTask", R"(Control the position of a frame.)")
      .def(nb::init<const mc_rbdyn::RobotFrame &, double, double>(), "frame"_a, "stiffness"_a = 2.0, "weight"_a = 500.0)

      .def(nb::init<const std::string &, const mc_rbdyn::Robots &, unsigned int, double, double>(), "bodyName"_a,
           "robots"_a, "robotIndex"_a, "stiffness"_a = 2.0, "weight"_a = 500.0)

      .def("reset", &PositionTask::reset, "Reset the task objective to the current body position.")

      .def("position", nb::overload_cast<>(&PositionTask::position, nb::const_),
           "Get the position target in world frame.")
      .def("position", nb::overload_cast<const Eigen::Vector3d &>(&PositionTask::position), "pos"_a,
           "Set the position target in world frame.")

      .def("bodyPoint", &PositionTask::bodyPoint, "Get the body point being controlled.");
}

} // namespace mc_rtc_python