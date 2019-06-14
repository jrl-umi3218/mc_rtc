#pragma once

#include <mc_rtc/GUIState.h>
#include <mc_trajectory/api.h>

#include <hpp/spline/exact_cubic.h>
#include <hpp/spline/spline_deriv_constraint.h>
#include <memory>
#include <vector>

namespace mc_trajectory
{

struct MC_TRAJECTORY_DLLAPI ExactCubic
{
public:
  using point_t = Eigen::Vector3d;
  using exact_cubic_t = spline::exact_cubic<double, double, 3, false, point_t>;
  using waypoint_t = std::pair<double, point_t>;
  using spline_deriv_constraint_t = spline::spline_deriv_constraint<double, double, 3, false, point_t>;
  using spline_constraints_t = spline_deriv_constraint_t::spline_constraints;

public:
  ExactCubic(const std::vector<waypoint_t> & waypoints,
             const point_t & init_vel,
             const point_t & init_acc,
             const point_t & end_vel,
             const point_t & end_acc);

  void waypoints(const std::vector<waypoint_t> & waypoints);
  void waypoint(size_t idx, const point_t & waypoint);
  void waypoint(size_t idx, const double t);
  const waypoint_t & waypoint(size_t idx) const;
  double waypointTime(size_t idx) const;
  const std::vector<waypoint_t> & waypoints() const;

  void target(const point_t & target);
  const point_t & target() const;

  const point_t & init_vel() const;
  const point_t & init_acc() const;
  const point_t & end_vel() const;
  const point_t & end_acc() const;

  std::vector<std::vector<Eigen::Vector3d>> splev(const std::vector<double> & t, unsigned int der = 0);
  std::vector<Eigen::Vector3d> sampleTrajectory(unsigned samples);

  void samplingPoints(const unsigned s);
  unsigned samplingPoints() const;

  void addToGUI(mc_rtc::gui::StateBuilder & gui, const std::vector<std::string> & category);

private:
  std::unique_ptr<spline_deriv_constraint_t> spline_;
  std::vector<waypoint_t> waypoints_;
  spline_constraints_t constraints_;

  bool needsUpdate_ = false;
  size_t samplingPoints_ = 10;
  std::vector<Eigen::Vector3d> samples_;
};

} // namespace mc_trajectory
