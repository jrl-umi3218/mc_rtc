#include <mc_tasks/ComplianceTask.h>
#include <mc_rbdyn/Robots.h>

#include <nanobind/nanobind.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;
using namespace nb::literals;

namespace mc_rtc_python
{

void bind_ComplianceTask(nb::module_ & m)
{
  using mc_tasks::force::ComplianceTask;

  nb::class_<ComplianceTask>(m, "ComplianceTask",
      R"(Force-compliant end-effector control task.

This task uses a wrench sensor to drive an end-effector until a target
force/torque level is reached, using a compliant behavior.)")

    /* --- Constructors --------------------------------------------------- */

    .def(nb::init<const mc_rbdyn::Robots &,
                  unsigned int,
                  const std::string &,
                  double,
                  const Eigen::Matrix6d &,
                  double,
                  double,
                  double,
                  double,
                  std::pair<double,double>,
                  std::pair<double,double>>(),
        "robots"_a,
        "robotIndex"_a,
        "body"_a,
        "timestep"_a,
        "dof"_a = Eigen::Matrix6d::Identity(),
        "stiffness"_a = 5.0,
        "weight"_a = 1000.0,
        "forceThresh"_a = 3.0,
        "torqueThresh"_a = 1.0,
        "forceGain"_a = ComplianceTask::defaultFGain,
        "torqueGain"_a = ComplianceTask::defaultTGain,
        R"(Constructor with DoF restriction matrix.)")

    .def(nb::init<const mc_rbdyn::Robots &,
                  unsigned int,
                  const std::string &,
                  double,
                  double,
                  double,
                  double,
                  double,
                  std::pair<double,double>,
                  std::pair<double,double>>(),
        "robots"_a,
        "robotIndex"_a,
        "body"_a,
        "timestep"_a,
        "stiffness"_a = 5.0,
        "weight"_a = 1000.0,
        "forceThresh"_a = 3.0,
        "torqueThresh"_a = 1.0,
        "forceGain"_a = ComplianceTask::defaultFGain,
        "torqueGain"_a = ComplianceTask::defaultTGain,
        R"(Constructor without DoF restriction matrix.)")

    /* --- Methods -------------------------------------------------------- */

    .def("reset", &ComplianceTask::reset,
        R"(Reset the target to the current end-effector position.)")

    .def("getFilteredWrench", &ComplianceTask::getFilteredWrench,
        R"(Return the filtered wrench used internally.)")

    .def("setTargetWrench", &ComplianceTask::setTargetWrench,
        "wrench"_a,
        R"(Set the target wrench.)")

    .def("getTargetWrench", &ComplianceTask::getTargetWrench,
        R"(Return the current target wrench.)")

    .def_prop_rw(
        "stiffness",
        [](ComplianceTask & t) { return t.stiffness(); },
        [](ComplianceTask & t, double s) { t.stiffness(s); },
        R"(Task stiffness.)")

    .def_prop_rw(
        "weight",
        [](ComplianceTask & t) { return t.weight(); },
        [](ComplianceTask & t, double w) { t.weight(w); },
        R"(Task weight.)")

    .def_prop_rw(
        "forceThresh",
        [](ComplianceTask & t) { return t.forceThresh(); },
        [](ComplianceTask & t, double v) { t.forceThresh(v); },
        R"(Force threshold.)")

    .def_prop_rw(
        "torqueThresh",
        [](ComplianceTask & t) { return t.torqueThresh(); },
        [](ComplianceTask & t, double v) { t.torqueThresh(v); },
        R"(Torque threshold.)")

    .def_prop_rw(
        "forceGain",
        [](ComplianceTask & t) { return t.forceGain(); },
        [](ComplianceTask & t, std::pair<double,double> g) { t.forceGain(g); },
        R"(Force PD gains (P, D).)")

    .def_prop_rw(
        "torqueGain",
        [](ComplianceTask & t) { return t.torqueGain(); },
        [](ComplianceTask & t, std::pair<double,double> g) { t.torqueGain(g); },
        R"(Torque PD gains (P, D).)")

    /* --- DoF selection -------------------------------------------------- */

    .def_prop_rw(
        "dof",
        [](ComplianceTask & t) { return t.dof(); },
        [](ComplianceTask & t, const Eigen::Matrix6d & M) { t.dof(M); },
        R"(6×6 DoF selection matrix.)")

    /* --- Dimension weights --------------------------------------------- */

    .def("dimWeight",
         nb::overload_cast<const Eigen::VectorXd &>(&ComplianceTask::dimWeight),
         "dimW"_a,
         R"(Set dimension-specific weights.)")

    .def("dimWeight",
         nb::overload_cast<>(&ComplianceTask::dimWeight, nb::const_),
         R"(Return dimension-specific weights.)")

    /* --- Joint selection ------------------------------------------------ */

    .def("selectActiveJoints",
        &ComplianceTask::selectActiveJoints,
        "solver"_a,
        "activeJointsName"_a,
        "activeDofs"_a = std::map<std::string,
                                 std::vector<std::array<int,2>>>{},
        R"(Select active joints for this task.)")

    .def("selectUnactiveJoints",
        &ComplianceTask::selectUnactiveJoints,
        "solver"_a,
        "unactiveJointsName"_a,
        "unactiveDofs"_a = std::map<std::string,
                                    std::vector<std::array<int,2>>>{},
        R"(Select unactive joints for this task.)")

    .def("resetJointsSelector",
        &ComplianceTask::resetJointsSelector,
        "solver"_a,
        R"(Reset joint selection.)")

    /* --- Task evaluation ------------------------------------------------ */

    .def("eval", &ComplianceTask::eval,
        R"(Return the evaluated wrench error.)")

    .def("speed", &ComplianceTask::speed,
        R"(Return body velocity as 6D vector.)");
}

} // namespace mc_rtc_python
