/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_trajectory/InterpolatedRotation.h>

namespace mc_trajectory
{

using waypoint_t = InterpolatedRotation::waypoint_t;

InterpolatedRotation::InterpolatedRotation(double duration,
                                           const Eigen::Matrix3d & start,
                                           const Eigen::Matrix3d & target,
                                           const std::vector<waypoint_t> & waypoints)
: Spline<Eigen::Matrix3d, std::vector<std::pair<double, Eigen::Matrix3d>>>(duration, start, target, waypoints)
{
}

void InterpolatedRotation::update() {}

void InterpolatedRotation::waypoint(size_t idx, const Eigen::Matrix3d & ori)
{
  if(idx >= waypoints_.size())
  {
    mc_rtc::log::error_and_throw("No waypoint with index {}", idx);
  }
  waypoints_[idx].second = ori;
  needsUpdate_ = true;
}

const waypoint_t & InterpolatedRotation::waypoint(size_t idx) const
{
  if(idx >= waypoints_.size())
  {
    mc_rtc::log::error_and_throw("No waypoint with index {}", idx);
  }
  return waypoints_[idx];
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
