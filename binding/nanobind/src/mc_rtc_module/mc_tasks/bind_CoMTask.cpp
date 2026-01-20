#include <mc_rbdyn/Robots.h>
#include <mc_tasks/CoMTask.h>

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;
using namespace nb::literals;

namespace mc_rtc_python
{

void bind_CoMTask(nb::module_ & m)
{
  using mc_tasks::CoMTask;
  nb::class_<CoMTask, mc_tasks::TrajectoryTaskGeneric>(m, "CoMTask",
                                                       R"(Task for controlling the robot's Center of Mass (CoM).)")

      .def(nb::init<const mc_rbdyn::Robots &, unsigned int, double, double>(), "robots"_a, "robotIndex"_a,
           "stiffness"_a = 5.0, "weight"_a = 100.0, R"(Create a CoMTask to control the selected robot's CoM.)")

      .def("reset", &CoMTask::reset, R"(Reset the CoM task target to the current CoM position.)")

      .def("move_com", &CoMTask::move_com, "com"_a, R"(Increment the CoM target by the given 3D vector.)")

      .def("com", nb::overload_cast<const Eigen::Vector3d &>(&CoMTask::com), "com"_a,
           R"(Set the CoM target to the given position.)")

      .def("com", nb::overload_cast<>(&CoMTask::com, nb::const_), nb::rv_policy::reference_internal,
           R"(Return the current CoM target.)")

      .def("actual", &CoMTask::actual, nb::rv_policy::reference_internal,
           R"(Return the actual CoM position of the robot.)")

      .def("load", &CoMTask::load, "solver"_a, "config"_a, R"(Load CoM task parameters from a configuration.)");
}

} // namespace mc_rtc_python
