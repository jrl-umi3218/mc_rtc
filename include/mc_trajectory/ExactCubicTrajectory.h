#pragma once

#include <mc_trajectory/api.h>
#include <spline/exact_cubic.h>
#include <spline/spline_deriv_constraint.h>
#include <vector>
#include <memory>

namespace mc_trajectory
{


struct MC_TRAJECTORY_DLLAPI ExactCubicTrajectory
{
public:
  using point_t = Eigen::Vector3d;
  using exact_cubic_t = spline::exact_cubic<double, double, 3, true, point_t>;
  using Waypoint = std::pair<double, point_t>;
  using T_Waypoint = std::vector<Waypoint>;
  using spline_deriv_constraint_t = spline::spline_deriv_constraint<double, double, 3, true, point_t> ;
  using spline_constraints_t = spline_deriv_constraint_t::spline_constraints;
public:
  ExactCubicTrajectory(const T_Waypoint & controlPoints, double duration, unsigned int order = 4);
  std::vector<std::vector<Eigen::Vector3d>> splev(const std::vector<double> & t, unsigned int der = 0);
  std::vector<Eigen::Vector3d> sampleTrajectory(unsigned samples);

private:
  double duration;
  unsigned int p;
  std::shared_ptr<spline_deriv_constraint_t> spline;
};

} // namespace mc_trajectory
