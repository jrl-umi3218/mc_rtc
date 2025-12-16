#include <mc_tasks/BSplineTrajectoryTask.h>
#include <mc_tasks/SplineTrajectoryTask.h>
#include <mc_rbdyn/RobotFrame.h>
#include <mc_rbdyn/Robots.h>
#include <mc_trajectory/BSpline.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/pair.h>
#include <nanobind/eigen/dense.h>
#include <SpaceVecAlg/SpaceVecAlg>

namespace nb = nanobind;
using namespace nb::literals;

namespace mc_rtc_python
{

void bind_BSplineTrajectoryTask(nb::module_ & m)
{
  using mc_tasks::BSplineTrajectoryTask;
  using mc_tasks::SplineTrajectoryTask;
  using waypoints_t = mc_trajectory::BSpline::waypoints_t;
  
  // First, ensure the base class is bound (if not already done elsewhere)
  // This may need to be done in a separate binding file
  
  // ------------------------------------------------------------------
  // BSplineTrajectoryTask
  // ------------------------------------------------------------------
  nb::class_<BSplineTrajectoryTask, SplineTrajectoryTask<BSplineTrajectoryTask>>(
    m, "BSplineTrajectoryTask",
    R"(
Track a B-spline curve with a robot surface.

This task handles trajectory tracking along a B-spline curve, managing task 
target updates, orientation waypoint handling, and all functionalities from 
TrajectoryTaskGeneric.
)")
    // Constructor with RobotFrame
    .def(nb::init<const mc_rbdyn::RobotFrame &,
                  double,
                  double,
                  double,
                  const sva::PTransformd &,
                  const waypoints_t &,
                  const std::vector<std::pair<double, Eigen::Matrix3d>> &>(),
         "frame"_a,
         "duration"_a,
         "stiffness"_a,
         "weight"_a,
         "target"_a,
         "posWp"_a = waypoints_t{},
         "oriWp"_a = std::vector<std::pair<double, Eigen::Matrix3d>>{},
         R"(
Creates a trajectory that follows a B-spline curve.

Parameters
----------
frame : mc_rbdyn.RobotFrame
    Control frame
duration : float
    Duration of motion (time to go from current position to curve's final point)
stiffness : float
    Stiffness of the underlying TrajectoryTask (position and orientation)
weight : float
    Task weight
target : sva.PTransformd
    Final world pose to reach
posWp : list of Vector3d, optional
    Waypoints in position
oriWp : list of (float, Matrix3d) pairs, optional
    Waypoints in orientation specified as pairs of (time, orientation)
)")
    
    // Constructor with Robots
    .def(nb::init<const mc_rbdyn::Robots &,
                  unsigned int,
                  const std::string &,
                  double,
                  double,
                  double,
                  const sva::PTransformd &,
                  const waypoints_t &,
                  const std::vector<std::pair<double, Eigen::Matrix3d>> &>(),
         "robots"_a,
         "robotIndex"_a,
         "surfaceName"_a,
         "duration"_a,
         "stiffness"_a,
         "weight"_a,
         "target"_a,
         "posWp"_a = waypoints_t{},
         "oriWp"_a = std::vector<std::pair<double, Eigen::Matrix3d>>{},
         R"(
Creates a trajectory that follows a B-spline curve.

Parameters
----------
robots : mc_rbdyn.Robots
    Robots controlled by the task
robotIndex : int
    Which robot is controlled
surfaceName : str
    Surface controlled by the task (should belong to the controlled robot)
duration : float
    Duration of motion (time to go from current position to curve's final point)
stiffness : float
    Stiffness of the underlying TrajectoryTask (position and orientation)
weight : float
    Task weight
target : sva.PTransformd
    Final world pose to reach
posWp : list of Vector3d, optional
    Waypoints in position
oriWp : list of (float, Matrix3d) pairs, optional
    Waypoints in orientation specified as pairs of (time, orientation)
)")
    
    // Access to underlying spline (const)
    .def("spline",
         nb::overload_cast<>(&BSplineTrajectoryTask::spline, nb::const_),
         nb::rv_policy::reference_internal,
         R"(Get const reference to the underlying B-spline.)")
    
    // Set position waypoints
    .def("posWaypoints",
         &BSplineTrajectoryTask::posWaypoints,
         "posWp"_a,
         R"(
Set control points for the B-spline curve (position).

Parameters
----------
posWp : list of Vector3d
    Vector of position control points for the curve.
    Should not include the starting and target position (use target() instead).
)");
}

} // namespace mc_rtc_python