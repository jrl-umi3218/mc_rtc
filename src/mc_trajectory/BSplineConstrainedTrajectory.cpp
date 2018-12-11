#include <mc_trajectory/BSplineConstrainedTrajectory.h>
#include <mc_rtc/logging.h>

namespace mc_trajectory
{

BSplineConstrainedTrajectory::BSplineConstrainedTrajectory(const std::vector<point_t> & controlPoints,
                                     double duration,
                                     unsigned int order)
: duration(duration), p(order), spline(controlPoints.begin(), controlPoints.end(), duration)
{
}

std::vector<std::vector<point_t>> BSplineConstrainedTrajectory::splev(const std::vector<double> & t, unsigned int der)
{
  std::vector<std::vector<point_t>> res(0);
  res.reserve(t.size());
  for(const auto & ti : t)
  {
    std::vector<Eigen::Vector3d> pts;
    pts.reserve(der + 1);
    for (std::size_t order = 0; order <= der; ++order) {
      pts.push_back(spline.derivate(ti, order));
    }
    res.push_back(pts);
  }
  return res;
}

} // namespace mc_trajectory
