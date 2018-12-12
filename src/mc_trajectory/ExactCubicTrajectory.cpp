#include <mc_rtc/logging.h>
#include <mc_trajectory/ExactCubicTrajectory.h>

namespace mc_trajectory
{

using point_t = ExactCubicTrajectory::point_t;
using exact_cubic_t = ExactCubicTrajectory::exact_cubic_t;
using Waypoint = ExactCubicTrajectory::Waypoint;
using T_Waypoint = ExactCubicTrajectory::T_Waypoint;
using spline_deriv_constraint_t = ExactCubicTrajectory::spline_deriv_constraint_t;
using spline_constraints_t = ExactCubicTrajectory::spline_constraints_t;

ExactCubicTrajectory::ExactCubicTrajectory(const T_Waypoint & waypoints, double duration, unsigned int order)
: duration(duration), p(order)
{
  LOG_INFO("ExactCubicTrajectory");
  spline_constraints_t constraints;
  constraints.init_vel = point_t(0., 0., 0.); // point_t(-1,-2,-3);
  constraints.init_acc = point_t(0., 0., 0.); // point_t(-4,-4,-6);
  constraints.end_vel = point_t(0., 0., .1);
  constraints.end_acc = point_t(0., 0., 0.);

  spline = std::make_shared<spline_deriv_constraint_t>(waypoints.begin(), waypoints.end(), constraints);
  LOG_INFO("spline created");
}

std::vector<std::vector<point_t>> ExactCubicTrajectory::splev(const std::vector<double> & t, unsigned int der)
{
  LOG_INFO("splev at t=" << t[0]);
  std::vector<std::vector<point_t>> res(0);
  res.reserve(t.size());
  for(const auto & ti : t)
  {
    std::vector<Eigen::Vector3d> pts;
    pts.reserve(der + 1);
    for(std::size_t order = 0; order <= der; ++order)
    {
      pts.push_back(spline->derivate(ti, order));
    }
    res.push_back(pts);
  }
  LOG_INFO("splev success");
  return res;
}

std::vector<Eigen::Vector3d> ExactCubicTrajectory::sampleTrajectory(unsigned samples)
{
  std::vector<Eigen::Vector3d> traj;
  traj.resize(samples);
  // Evaluate trajectory for display
  for(unsigned i = 0; i < samples; ++i)
  {
    auto time = duration * i / (samples - 1);
    auto res = splev({time}, 0);
    Eigen::Vector3d & pos = res[0][0];
    traj[i] = pos;
  }
  return traj;
}

} // namespace mc_trajectory
