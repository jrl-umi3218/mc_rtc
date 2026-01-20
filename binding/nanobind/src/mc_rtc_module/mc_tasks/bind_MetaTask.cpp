#include <mc_tasks/MetaTask.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;
using namespace nb::literals;

namespace mc_rtc_python
{

void bind_MetaTask(nb::module_ & m)
{
  nb::class_<mc_tasks::MetaTask>(m, "MetaTask")

      .def_prop_ro("type", &mc_tasks::MetaTask::type, "Get the type of the task")

      .def("name", nb::overload_cast<>(&mc_tasks::MetaTask::name, nb::const_), "Get the name of the task")
      .def("name", nb::overload_cast<const std::string &>(&mc_tasks::MetaTask::name), "name"_a,
           "Set the name of the task")

      .def("reset", &mc_tasks::MetaTask::reset, "Reset the task")
      .def("eval", &mc_tasks::MetaTask::eval, "Returns the task error")
      .def("speed", &mc_tasks::MetaTask::speed, "Returns the task velocity")

      .def("dimWeight", nb::overload_cast<const Eigen::VectorXd &>(&mc_tasks::MetaTask::dimWeight), "dimW"_a)
      .def("dimWeight", nb::overload_cast<>(&mc_tasks::MetaTask::dimWeight, nb::const_))

      .def("iterInSolver", &mc_tasks::MetaTask::iterInSolver, "Iterations since added to solver")
      .def("backend", &mc_tasks::MetaTask::backend, "Get the QPSolver backend used by this task")

      .def("resetJointsSelector", &mc_tasks::MetaTask::resetJointsSelector, "solver"_a);
}

} // namespace mc_rtc_python