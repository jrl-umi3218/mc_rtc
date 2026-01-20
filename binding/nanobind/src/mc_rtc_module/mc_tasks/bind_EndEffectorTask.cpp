#include <mc_rbdyn/Robots.h>
#include <mc_tasks/EndEffectorTask.h>

#include "mc_tasks/MetaTask.h"
#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;
using namespace nb::literals;

namespace mc_rtc_python
{

void bind_EndEffectorTask(nb::module_ & m)
{
  using mc_tasks::EndEffectorTask;

  nb::class_<EndEffectorTask, mc_tasks::MetaTask>(m, "EndEffectorTask",
                                                  R"(Controls an end-effector of the robot.

This task wraps position and orientation tasks for a robot end-effector.
The objective is defined in the world frame.)")

      // Constructors
      .def(nb::init<const mc_rbdyn::RobotFrame &, double, double>(), "frame"_a, "stiffness"_a = 2.0,
           "weight"_a = 1000.0, R"(Create an EndEffectorTask from a control frame.)")

      .def(nb::init<const std::string &, const mc_rbdyn::Robots &, unsigned int, double, double>(), "bodyName"_a,
           "robots"_a, "robotIndex"_a, "stiffness"_a = 2.0, "weight"_a = 1000.0,
           R"(Create an EndEffectorTask targeting a body by name.)")

      .def_prop_ro("positionTask", [](mc_tasks::EndEffectorTask & t) { return t.positionTask; })
      .def_prop_ro("orientationTask", [](mc_tasks::EndEffectorTask & t) { return t.orientationTask; })

      // Reset
      .def("reset", &EndEffectorTask::reset, R"(Reset the task objective to the current end-effector position.)")

      // Target manipulation
      .def("add_ef_pose", &EndEffectorTask::add_ef_pose, "dtr"_a,
           R"(Increment the end-effector target by the given transform.)")

      .def("set_ef_pose", &EndEffectorTask::set_ef_pose, "tf"_a,
           R"(Set the end-effector target to the given transform.)")

      .def("get_ef_pose", &EndEffectorTask::get_ef_pose, R"(Get the current end-effector target transform.)")

      // Dimensional weights
      .def("dimWeight", nb::overload_cast<const Eigen::VectorXd &>(&EndEffectorTask::dimWeight), "dimW"_a,
           R"(Set the dimensional weights for the task.)")

      .def("dimWeight", nb::overload_cast<>(&EndEffectorTask::dimWeight, nb::const_),
           R"(Get the dimensional weights for the task.)")

      // Name
      .def("name", nb::overload_cast<>(&EndEffectorTask::name, nb::const_), R"(Get the task name.)")
      .def("name", nb::overload_cast<const std::string &>(&EndEffectorTask::name), "name"_a, R"(Set the task name.)");
}

} // namespace mc_rtc_python
