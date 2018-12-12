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

ExactCubicTrajectory::ExactCubicTrajectory(const T_Waypoint & waypoints,
                                           const point_t & init_vel,
                                           const point_t & init_acc,
                                           const point_t & end_vel,
                                           const point_t & end_acc)
{
  spline_constraints_t constraints;
  constraints.init_vel = init_vel;
  constraints.init_acc = init_acc;
  constraints.end_vel = end_vel;
  constraints.end_acc = end_acc;

  spline = std::make_shared<spline_deriv_constraint_t>(waypoints.begin(), waypoints.end(), constraints);
}

std::vector<std::vector<point_t>> ExactCubicTrajectory::splev(const std::vector<double> & t, unsigned int der)
{
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
  return res;
}

std::vector<Eigen::Vector3d> ExactCubicTrajectory::sampleTrajectory(unsigned samples)
{
  std::vector<Eigen::Vector3d> traj;
  traj.resize(samples);
  // Evaluate trajectory for display
  for(unsigned i = 0; i < samples; ++i)
  {
    auto time = spline->min() + (spline->max() - spline->min()) * i / (samples - 1);
    auto res = splev({time}, 0);
    Eigen::Vector3d & pos = res[0][0];
    traj[i] = pos;
  }
  return traj;
}

} // namespace mc_trajectory
