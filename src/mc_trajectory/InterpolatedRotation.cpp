#include <mc_trajectory/InterpolatedRotation.h>

namespace mc_trajectory
{

using waypoint_t = InterpolatedRotation::waypoint_t;

InterpolatedRotation::InterpolatedRotation(double duration,
                                           const Eigen::Matrix3d & start,
                                           const Eigen::Matrix3d & target,
                                           const std::vector<waypoint_t> & waypoints)
: duration_(duration), start_(start), target_(target), waypoints_(waypoints), needsUpdate_(true)
{
}

void InterpolatedRotation::waypoints(const std::vector<waypoint_t> & waypoints)
{
  waypoints_ = waypoints;
  needsUpdate_ = true;
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
  needsUpdate_ = true;
}

const waypoint_t & InterpolatedRotation::waypoint(size_t idx) const
{
  if(idx >= waypoints_.size())
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "No waypoint with index " << idx);
  }
  return waypoints_[idx];
}

void InterpolatedRotation::start(const Eigen::Matrix3d & ori)
{
  start_ = ori;
  needsUpdate_ = true;
}

const Eigen::Matrix3d & InterpolatedRotation::start() const
{
  return start_;
}

void InterpolatedRotation::target(const Eigen::Matrix3d & ori)
{
  target_ = ori;
  needsUpdate_ = true;
}

const Eigen::Matrix3d & InterpolatedRotation::target() const
{
  return target_;
}

Eigen::Matrix3d InterpolatedRotation::eval(double t)
{
  if(needsUpdate_)
  {
    all_waypoints_.clear();
    all_waypoints_.reserve(waypoints_.size() + 2);
    all_waypoints_.push_back(std::make_pair(0, start_));
    for(const auto & wp : waypoints_)
    {
      all_waypoints_.push_back(wp);
    }
    all_waypoints_.push_back(std::make_pair(duration_, target_));
    needsUpdate_ = false;
  }

  // Find waypoint following the current time t
  unsigned i = 1;
  for(; i < all_waypoints_.size(); ++i)
  {
    if(t - all_waypoints_[i].first <= 0) break;
  }
  unsigned end = std::min(static_cast<unsigned>(all_waypoints_.size() - 1), i);
  unsigned start = i - 1;

  double ts = all_waypoints_[start].first;
  double te = all_waypoints_[end].first;
  double duration = te - ts;
  Eigen::Quaterniond qfrom(all_waypoints_[start].second);
  Eigen::Quaterniond qto(all_waypoints_[end].second);
  return Eigen::Matrix3d(qfrom.slerp((t - ts) / duration, qto));
}
} // namespace mc_trajectory
