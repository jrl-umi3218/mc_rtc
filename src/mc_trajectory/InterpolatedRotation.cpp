#include <mc_trajectory/InterpolatedRotation.h>

namespace mc_trajectory
{

InterpolatedRotation::InterpolatedRotation(const std::vector<std::pair<double, Eigen::Matrix3d>> & waypoints)
: waypoints_(waypoints)
{
}

std::vector<std::pair<double, Eigen::Matrix3d>> & InterpolatedRotation::waypoints()
{
  return waypoints_;
}

void InterpolatedRotation::waypoints(const std::vector<std::pair<double, Eigen::Matrix3d>> & waypoints)
{
  if(waypoints.size() < 2)
  {
    LOG_ERROR_AND_THROW(std::runtime_error,
                        "There should be at least two orientation waypoints for the start and final orientations");
  }
  waypoints_ = waypoints;
}

void InterpolatedRotation::target(const Eigen::Matrix3d & ori)
{
  waypoints_.back().second = ori;
}

const Eigen::Matrix3d & InterpolatedRotation::target() const
{
  return waypoints_.back().second;
}

Eigen::Matrix3d InterpolatedRotation::eval(double t)
{
  // Find waypoint following the current time t
  unsigned i = 1;
  for(; i < waypoints_.size(); ++i)
  {
    if(t - waypoints_[i].first <= 0) break;
  }
  unsigned end = std::min(static_cast<unsigned>(waypoints_.size() - 1), i);
  unsigned start = std::max(0u, i - 1);

  double ts = waypoints_[start].first;
  double te = waypoints_[end].first;
  double duration = te - ts;
  Eigen::Quaterniond qfrom(waypoints_[start].second);
  Eigen::Quaterniond qto(waypoints_[end].second);
  return Eigen::Matrix3d(qfrom.slerp((t - ts) / duration, qto));
}
} // namespace mc_trajectory
