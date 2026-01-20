#include <mc_tasks/ImpedanceTask.h>
#include <mc_rbdyn/RobotFrame.h>

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;
using namespace nb::literals;

namespace mc_rtc_python
{

void bind_ImpedanceTask(nb::module_ & m)
{
  using mc_tasks::force::ImpedanceTask;
  using mc_tasks::force::ImpedanceGains;

  nb::class_<ImpedanceTask>(m, "ImpedanceTask",
      R"(Impedance control of an end-effector.

This task wraps TransformTask and applies impedance control to the end-effector,
allowing compliance, force control, and hold modes.)")

    .def(nb::init<const std::string &,
                  const mc_rbdyn::Robots &,
                  unsigned int,
                  double,
                  double>(),
         "surfaceName"_a,
         "robots"_a,
         "robotIndex"_a,
         "stiffness"_a = 5.0,
         "weight"_a = 1000.0,
         R"(Create ImpedanceTask by surface name.)")

    .def(nb::init<const mc_rbdyn::RobotFrame &,
                  double,
                  double>(),
         "frame"_a,
         "stiffness"_a = 5.0,
         "weight"_a = 1000.0,
         R"(Create ImpedanceTask by RobotFrame.)")

    .def("reset", &ImpedanceTask::reset,
         R"(Reset the task to the current surface pose and zero velocity/acceleration.)")

    .def_prop_rw("gains",
      [](const ImpedanceTask & it) -> const ImpedanceGains & { return it.gains(); },
      [](ImpedanceTask & it, const ImpedanceGains & g) { it.gains() = g; },
      R"(Access or modify the impedance gains.)")

    .def_prop_rw("targetPose",
      [](const ImpedanceTask & it) -> const sva::PTransformd & { return it.targetPose(); },
      [](ImpedanceTask & it, const sva::PTransformd & pose) { it.targetPose(pose); },
      R"(Target pose in world frame.)")

    .def_prop_rw("targetVel",
      [](const ImpedanceTask & it) -> const sva::MotionVecd & { return it.targetVel(); },
      [](ImpedanceTask & it, const sva::MotionVecd & vel) { it.targetVel(vel); },
      R"(Target velocity in world frame.)")

    .def_prop_rw("targetAccel",
      [](const ImpedanceTask & it) -> const sva::MotionVecd & { return it.targetAccel(); },
      [](ImpedanceTask & it, const sva::MotionVecd & accel) { it.targetAccel(accel); },
      R"(Target acceleration in world frame.)")

    .def_prop_ro("compliancePose",
                           &ImpedanceTask::compliancePose,
                           R"(Compliance pose in world frame.)")

    .def_prop_rw("targetWrench",
      [](const ImpedanceTask & it) -> const sva::ForceVecd & { return it.targetWrench(); },
      [](ImpedanceTask & it, const sva::ForceVecd & wrench) { it.targetWrench(wrench); },
      R"(Target wrench in surface frame.)")

    .def_prop_ro("measuredWrench",
                           &ImpedanceTask::measuredWrench,
                           R"(Measured wrench in surface frame.)")

    .def_prop_ro("filteredMeasuredWrench",
                           &ImpedanceTask::filteredMeasuredWrench,
                           R"(Filtered measured wrench in surface frame.)")

    .def_prop_rw("cutoffPeriod",
        [](const ImpedanceTask & it) { return it.cutoffPeriod(); },
        [](ImpedanceTask & it, double cutoffPeriod) { it.cutoffPeriod(cutoffPeriod); },
        R"(Low-pass filter cutoff period for measured wrench.)")

    .def_prop_rw("hold",
        [](const ImpedanceTask & it) { return it.hold(); },
        [](ImpedanceTask & it, bool hold) { it.hold(hold); },
        R"(Enable/disable hold mode.)")

    .def("load", &ImpedanceTask::load,
         "solver"_a,
         "config"_a,
         R"(Load parameters from a configuration object.)");
}

} // namespace mc_rtc_python
