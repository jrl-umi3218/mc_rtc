#include <mc_tasks/SplineTrajectoryTask.h>
#include <mc_solver/QPSolver.h>

#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;
using namespace nb::literals;

namespace mc_rtc_python
{

template<typename Derived>
void bind_SplineTrajectoryTask(nb::module_ & m, const char * name)
{
  using mc_tasks::SplineTrajectoryTask;
  using Sequence6d = std::vector<std::pair<double, Eigen::Vector6d>>;
  using OriWp = std::vector<std::pair<double, Eigen::Matrix3d>>;

  nb::class_<SplineTrajectoryTask<Derived>>(m, name,
      R"(Generic task tracking a curve in position and orientation.
      
Tracks a trajectory curve with optional orientation waypoints. Gains can be set
per-dimension or interpolated over time.)")
    
    // Constructor
    .def(nb::init<const mc_rbdyn::RobotFrame &,
                  double,
                  double,
                  double,
                  const Eigen::Matrix3d &,
                  const OriWp &>(),
         "frame"_a,
         "duration"_a,
         "stiffness"_a,
         "weight"_a,
         "target"_a,
         "oriWaypoints"_a = OriWp(),
         R"(Create a SplineTrajectoryTask.

Parameters
----------
frame : mc_rbdyn.RobotFrame
duration : float
stiffness : float
weight : float
target : Eigen::Matrix3d
oriWaypoints : list of [time, Eigen::Matrix3d], optional
)")

    // Load
    .def("load", &SplineTrajectoryTask<Derived>::load,
         "solver"_a, "config"_a)

    // Orientation waypoints
    .def("oriWaypoints", &SplineTrajectoryTask<Derived>::oriWaypoints,
         "waypoints"_a)

    // Target setters/getters
    .def("target",
         nb::overload_cast<const sva::PTransformd &>(&SplineTrajectoryTask<Derived>::target),
         "target"_a)
    .def("target",
         nb::overload_cast<>(&SplineTrajectoryTask<Derived>::target, nb::const_))

    // Display samples
    .def("displaySamples",
         nb::overload_cast<unsigned>(&SplineTrajectoryTask<Derived>::displaySamples),
         "samples"_a)
    .def("displaySamples",
         nb::overload_cast<>(&SplineTrajectoryTask<Derived>::displaySamples, nb::const_))

    // Pause
    .def("pause", nb::overload_cast<bool>(&SplineTrajectoryTask<Derived>::pause),
         "paused"_a)
    .def("pause", nb::overload_cast<>(&SplineTrajectoryTask<Derived>::pause, nb::const_))

    // Current time and duration
    .def_property_readonly("currentTime",
                           &SplineTrajectoryTask<Derived>::currentTime)
    .def_property_readonly("duration",
                           &SplineTrajectoryTask<Derived>::duration)

    // Gains: stiffness/damping setters
    .def("stiffness",
         nb::overload_cast<double>(&SplineTrajectoryTask<Derived>::stiffness),
         "stiffness"_a)
    .def("stiffness",
         nb::overload_cast<const Eigen::VectorXd &>(&SplineTrajectoryTask<Derived>::stiffness),
         "stiffness"_a)
    .def("damping",
         nb::overload_cast<double>(&SplineTrajectoryTask<Derived>::damping),
         "damping"_a)
    .def("damping",
         nb::overload_cast<const Eigen::VectorXd &>(&SplineTrajectoryTask<Derived>::damping),
         "damping"_a)
    .def("setGains",
         nb::overload_cast<double, double>(&SplineTrajectoryTask<Derived>::setGains),
         "stiffness"_a, "damping"_a)
    .def("setGains",
         nb::overload_cast<const Eigen::VectorXd &, const Eigen::VectorXd &>(&SplineTrajectoryTask<Derived>::setGains),
         "stiffness"_a, "damping"_a)

    // Dimensional weights
    .def("dimWeight",
         nb::overload_cast<const Eigen::VectorXd &>(&SplineTrajectoryTask<Derived>::dimWeight),
         "weights"_a)
    .def("dimWeight",
         nb::overload_cast<>(&SplineTrajectoryTask<Derived>::dimWeight, nb::const_))

    // Interpolation
    .def("dimWeightInterpolation", &SplineTrajectoryTask<Derived>::dimWeightInterpolation)
    .def("stiffnessInterpolation", &SplineTrajectoryTask<Derived>::stiffnessInterpolation)
    .def("dampingInterpolation", &SplineTrajectoryTask<Derived>::dampingInterpolation)

    // Evaluation
    .def("eval", &SplineTrajectoryTask<Derived>::eval)
    .def("evalTracking", &SplineTrajectoryTask<Derived>::evalTracking)

    // Completion criteria
    .def("buildCompletionCriteria", &SplineTrajectoryTask<Derived>::buildCompletionCriteria);
}

} // namespace mc_rtc_python
