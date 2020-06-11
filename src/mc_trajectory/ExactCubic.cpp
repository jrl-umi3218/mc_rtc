/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/logging.h>
#include <mc_trajectory/ExactCubic.h>

#include <mc_rtc/gui/NumberInput.h>
#include <mc_rtc/gui/Point3D.h>
#include <mc_rtc/gui/Trajectory.h>

namespace mc_trajectory
{
using point_t = ExactCubic::point_t;
using exact_cubic_t = ExactCubic::exact_cubic_t;
using waypoint_t = ExactCubic::waypoint_t;
using spline_deriv_constraint_t = ExactCubic::spline_deriv_constraint_t;
using spline_constraints_t = ExactCubic::spline_constraints_t;

ExactCubic::ExactCubic(double duration,
                       const point_t & start,
                       const point_t & target,
                       const std::vector<waypoint_t> & waypoints,
                       const point_t & init_vel,
                       const point_t & init_acc,
                       const point_t & end_vel,
                       const point_t & end_acc)
: Spline<point_t, std::vector<waypoint_t>>(duration, start, target, waypoints)
{
  this->constraints(init_vel, init_acc, end_vel, end_acc);
  this->update();
}

void ExactCubic::update()
{
  if(needsUpdate_)
  {
    std::vector<waypoint_t> waypoints;
    waypoints.reserve(waypoints_.size() + 2);
    waypoints.push_back(std::make_pair(0., start_));
    for(const auto & wp : waypoints_)
    {
      waypoints.push_back(wp);
    }
    waypoints.push_back(std::make_pair(duration_, target_));
    spline_.reset(new spline_deriv_constraint_t(waypoints.begin(), waypoints.end(), constraints_));
    samples_ = this->sampleTrajectory();
    needsUpdate_ = false;
  }
}

void ExactCubic::waypoint(size_t idx, const point_t & waypoint)
{
  if(idx >= waypoints_.size())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Cannot modify waypoint with index {}", idx);
  }
  waypoints_[idx].second = waypoint;
  needsUpdate_ = true;
}

void ExactCubic::waypoint(size_t idx, double t)
{
  if(idx >= waypoints_.size())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Cannot modify waypoint with index {}", idx);
  }
  waypoints_[idx].first = t;
  needsUpdate_ = true;
}

double ExactCubic::waypointTime(size_t idx) const
{
  if(idx >= waypoints_.size())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("No waypoint with index {}", idx);
  }
  return waypoints_[idx].first;
}

const waypoint_t & ExactCubic::waypoint(size_t idx) const
{
  if(idx >= waypoints_.size())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("No waypoint with index {}", idx);
  }
  return waypoints_[idx];
}

void ExactCubic::constraints(const point_t & init_vel,
                             const point_t & init_acc,
                             const point_t & end_vel,
                             const point_t & end_acc)
{
  constraints_.init_vel = init_vel;
  constraints_.init_acc = init_acc;
  constraints_.end_vel = end_vel;
  constraints_.end_acc = end_acc;
  needsUpdate_ = true;
}

const point_t & ExactCubic::init_vel() const
{
  return constraints_.init_vel;
}
const point_t & ExactCubic::init_acc() const
{
  return constraints_.init_acc;
}
const point_t & ExactCubic::end_vel() const
{
  return constraints_.end_vel;
}
const point_t & ExactCubic::end_acc() const
{
  return constraints_.end_acc;
}

std::vector<point_t> ExactCubic::splev(double t, unsigned int der)
{
  std::vector<Eigen::Vector3d> pts;
  pts.reserve(der + 1);
  for(std::size_t order = 0; order <= der; ++order)
  {
    pts.push_back(spline_->derivate(t, order));
  }
  return pts;
}

std::vector<Eigen::Vector3d> ExactCubic::sampleTrajectory()
{
  if(samplingPoints_ < 1)
  {
    mc_rtc::log::error("There should be at least 1 sample");
    ;
    return {};
  }
  std::vector<Eigen::Vector3d> traj;
  traj.resize(samplingPoints_);
  // Evaluate trajectory for display
  for(unsigned i = 0; i < samplingPoints_; ++i)
  {
    auto time = spline_->min() + (spline_->max() - spline_->min()) * i / (samplingPoints_ - 1);
    auto res = splev(time, 0);
    traj[i] = res[0];
  }
  return traj;
}

void ExactCubic::addToGUI(mc_rtc::gui::StateBuilder & gui, const std::vector<std::string> & category)
{
  gui.addElement(category,
                 mc_rtc::gui::Point3D("Target Position", [this]() -> const Eigen::Vector3d & { return target(); },
                                      [this](const Eigen::Vector3d & pos) { target(pos); }));

  gui.addElement(category, mc_rtc::gui::Trajectory("Trajectory", [this]() { return samples_; }));

  // Interactive control points (target is handled independently)
  std::vector<std::string> waypointCategory = category;
  waypointCategory.push_back("Position Control Points");
  for(unsigned int i = 0; i < waypoints_.size(); ++i)
  {
    gui.addElement(waypointCategory,
                   mc_rtc::gui::Point3D("Waypoint " + std::to_string(i),
                                        [this, i]() -> const Eigen::Vector3d & { return waypoint(i).second; },
                                        [this, i](const Eigen::Vector3d & pos) { waypoint(i, pos); }),
                   mc_rtc::gui::NumberInput("Time " + std::to_string(i), [this, i]() { return waypointTime(i); },
                                            [this, i](double t) { waypoint(i, t); }));
  }
}

} // namespace mc_trajectory
