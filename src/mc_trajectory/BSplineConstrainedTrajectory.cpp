#include <mc_rtc/logging.h>
#include <mc_trajectory/BSplineConstrainedTrajectory.h>

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
    for(std::size_t order = 0; order <= der; ++order)
    {
      pts.push_back(spline.derivate(ti, order));
    }
    res.push_back(pts);
  }
  return res;
}

std::vector<Eigen::Vector3d> BSplineConstrainedTrajectory::sampleTrajectory(unsigned samples)
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
