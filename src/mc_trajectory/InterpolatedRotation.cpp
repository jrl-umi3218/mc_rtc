#include <mc_trajectory/InterpolatedRotation.h>

namespace mc_trajectory
{

using waypoint_t = InterpolatedRotation::waypoint_t;

InterpolatedRotation::InterpolatedRotation(const std::vector<waypoint_t> & waypoints) : waypoints_(waypoints) {}

InterpolatedRotation::InterpolatedRotation() {}

void InterpolatedRotation::waypoints(const std::vector<waypoint_t> & waypoints)
{
  if(waypoints.size() < 2)
  {
    LOG_ERROR_AND_THROW(std::runtime_error,
                        "There should be at least two orientation waypoints for the start and final orientations");
  }
  waypoints_ = waypoints;
}

std::vector<waypoint_t> & InterpolatedRotation::waypoints()
{
  return waypoints_;
}

void InterpolatedRotation::waypoint(size_t idx, const Eigen::Matrix3d & ori)
{
  if(idx >= waypoints_.size())
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "No waypoint with index " << idx);
  }
  waypoints_[idx].second = ori;
}

const waypoint_t & InterpolatedRotation::waypoint(size_t idx) const
{
  if(idx >= waypoints_.size())
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "No waypoint with index " << idx);
  }
  return waypoints_[idx];
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
  if(waypoints_.size() < 2)
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "There should be at least two orientation waypoints!");
  }
  // Find waypoint following the current time t
  unsigned i = 1;
  for(; i < waypoints_.size(); ++i)
  {
    if(t - waypoints_[i].first <= 0) break;
  }
  unsigned end = std::min(static_cast<unsigned>(waypoints_.size() - 1), i);
  unsigned start = i - 1;

  double ts = waypoints_[start].first;
  double te = waypoints_[end].first;
  double duration = te - ts;
  Eigen::Quaterniond qfrom(waypoints_[start].second);
  Eigen::Quaterniond qto(waypoints_[end].second);
  return Eigen::Matrix3d(qfrom.slerp((t - ts) / duration, qto));
}
} // namespace mc_trajectory
