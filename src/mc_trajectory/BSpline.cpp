/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_trajectory/BSpline.h>

#include <sstream>

namespace mc_trajectory
{
using point_t = BSpline::point_t;
using waypoints_t = BSpline::waypoints_t;
using bezier_curve_t = BSpline::bezier_curve_t;

BSpline::BSpline(double duration, const point_t & start, const point_t & target, const waypoints_t & waypoints)
: duration_(duration), waypoints_(waypoints)
{
  this->start(start);
  this->target(target);
  this->waypoints(waypoints);
  this->update();
}

void BSpline::update()
{
  if(needsUpdate_)
  {
    // Waypoints including start and target position
    std::vector<point_t> waypoints;
    waypoints.reserve(waypoints_.size() + 2);
    waypoints.push_back(start_);
    for(const auto & wp : waypoints_)
    {
      waypoints.push_back(wp);
    }
    waypoints.push_back(target_);
    spline.reset(new bezier_curve_t(waypoints.begin(), waypoints.end(), duration_));
    samples_ = this->sampleTrajectory(samplingPoints_);
    needsUpdate_ = false;
  }
}

std::vector<Eigen::Vector3d> BSpline::splev(double t, unsigned int der)
{
  if(spline == nullptr)
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "Invalide BSpline: there should be at least two waypoints");
  }
  std::vector<Eigen::Vector3d> pts;
  pts.reserve(der + 1);
  for(std::size_t order = 0; order <= der; ++order)
  {
    pts.push_back(spline->derivate(t, order));
  }
  return pts;
}

std::vector<Eigen::Vector3d> BSpline::sampleTrajectory(unsigned samples)
{
  if(samples < 1)
  {
    LOG_ERROR("There should be at least 1 sample");
    return {};
  }
  std::vector<Eigen::Vector3d> traj;
  traj.resize(samples);
  // Evaluate trajectory for display
  for(unsigned i = 0; i < samples; ++i)
  {
    auto time = duration_ * i / (samples - 1);
    auto res = splev(time, 0);
    Eigen::Vector3d & pos = res[0];
    traj[i] = pos;
  }
  return traj;
}

void BSpline::waypoints(const waypoints_t & waypoints)
{
  waypoints_ = waypoints;
  needsUpdate_ = true;
}

const waypoints_t & BSpline::waypoints() const
{
  return waypoints_;
}

void BSpline::start(const point_t & pos)
{
  start_ = pos;
  needsUpdate_ = true;
}

const point_t & BSpline::start() const
{
  return start_;
}

void BSpline::target(const point_t & target)
{
  target_ = target;
  needsUpdate_ = true;
}

const point_t & BSpline::target() const
{
  return target_;
}

void BSpline::samplingPoints(const unsigned s)
{
  if(s != samplingPoints_)
  {
    needsUpdate_ = true;
  }
  samplingPoints_ = s;
}

unsigned BSpline::samplingPoints() const
{
  return samplingPoints_;
}

void BSpline::addToGUI(mc_rtc::gui::StateBuilder & gui, const std::vector<std::string> & category)
{
  gui.addElement(category,
                 mc_rtc::gui::Point3D("Target Position", [this]() -> const Eigen::Vector3d & { return target(); },
                                      [this](const Eigen::Vector3d & pos) { target(pos); }));

  // Display trajectory
  gui.addElement(category, mc_rtc::gui::Trajectory("Trajectory", [this]() { return samples_; }));

  // Interactive control points
  std::vector<std::string> waypointCategory = category;
  waypointCategory.push_back("Position Control Points");
  for(unsigned int i = 0; i < this->waypoints().size(); ++i)
  {
    gui.addElement(waypointCategory,
                   mc_rtc::gui::Point3D("Waypoint " + std::to_string(i),
                                        [this, i]() -> const Eigen::Vector3d & { return waypoints_[i]; },
                                        [this, i](const Eigen::Vector3d & pos) {
                                          waypoints_[i] = pos;
                                          this->waypoints(waypoints_);
                                        }));
  }
}

} // namespace mc_trajectory
