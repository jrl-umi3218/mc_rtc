#pragma once

#include <mc_trajectory/api.h>
#include <spline/bezier_curve.h>

namespace mc_trajectory
{

using point_t = Eigen::Vector3d;
using time_t = double;
using bezier_curve_t = spline::bezier_curve<time_t, double, 3, true, point_t>;

struct MC_TRAJECTORY_DLLAPI BSplineConstrainedTrajectory
{
public:
  BSplineConstrainedTrajectory(const std::vector<point_t> & controlPoints, double duration, unsigned int order = 4);
  // BSplineConstrainedTrajectory(const std::vector<Eigen::Vector3d> & controlPoints, const std::vector<Eigen::Vector3d> & derivatives, const std::vector<unsigned> & derivativeIndices, double duration, unsigned int order = 4);

  std::vector<std::vector<Eigen::Vector3d>> splev(const std::vector<double> & t, unsigned int der = 0);
  std::vector<Eigen::Vector3d> sampleTrajectory(unsigned samples);

private:
  double duration;
  unsigned int p;
  bezier_curve_t spline;
};

} // namespace mc_trajectory
