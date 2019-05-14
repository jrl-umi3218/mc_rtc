#pragma once

#include <mc_trajectory/api.h>

#include <hpp/spline/exact_cubic.h>
#include <hpp/spline/spline_deriv_constraint.h>
#include <memory>
#include <vector>

namespace mc_trajectory
{

using point_t = Eigen::Vector3d;
using exact_cubic_t = spline::exact_cubic<double, double, 3, false, point_t>;
using waypoint_t = std::pair<double, point_t>;
using spline_deriv_constraint_t = spline::spline_deriv_constraint<double, double, 3, false, point_t>;
using spline_constraints_t = spline_deriv_constraint_t::spline_constraints;

struct MC_TRAJECTORY_DLLAPI ExactCubicTrajectory
{
public:

public:
  ExactCubicTrajectory(const std::vector<waypoint_t> & waypoints,
                       const point_t & init_vel,
                       const point_t & init_acc,
                       const point_t & end_vel,
                       const point_t & end_acc);

  void waypoints(const std::vector<waypoint_t> & waypoints);
  const std::vector<waypoint_t> & waypoints() const;
  const point_t & init_vel() const;
  const point_t & init_acc() const;
  const point_t & end_vel() const;
  const point_t & end_acc() const;

  std::vector<std::vector<Eigen::Vector3d>> splev(const std::vector<double> & t, unsigned int der = 0);
  std::vector<Eigen::Vector3d> sampleTrajectory(unsigned samples);

private:
  std::shared_ptr<spline_deriv_constraint_t> spline_;
  std::vector<waypoint_t> waypoints_;
  spline_constraints_t constraints_;
};

} // namespace mc_trajectory
