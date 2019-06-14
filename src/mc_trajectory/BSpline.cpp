/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_trajectory/BSpline.h>

#include <sstream>

namespace mc_trajectory
{
using point_t = BSpline::point_t;
using t_point_t = BSpline::t_point_t;
using bezier_curve_t = BSpline::bezier_curve_t;

BSpline::BSpline(const t_point_t & controlPoints, double duration, unsigned int order)
: duration(duration), p(order), controlPoints_(controlPoints)
{
  spline.reset(new bezier_curve_t(controlPoints.begin(), controlPoints.end(), duration));
}

BSpline::BSpline(double duration, unsigned int order) : duration(duration), p(order) {}

void BSpline::update()
{
  if(needsUpdate_)
  {
    spline.reset(new bezier_curve_t(controlPoints_.begin(), controlPoints_.end(), duration));
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
    auto time = duration * i / (samples - 1);
    auto res = splev(time, 0);
    Eigen::Vector3d & pos = res[0];
    traj[i] = pos;
  }
  return traj;
}

void BSpline::controlPoints(const t_point_t & waypoints)
{
  if(waypoints.size() < 2)
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "There should be at least two waypoints");
  }
  controlPoints_ = waypoints;
  needsUpdate_ = true;
}

void BSpline::target(const point_t & target)
{
  controlPoints_.back() = target;
  needsUpdate_ = true;
}

const point_t & BSpline::target() const
{
  return controlPoints_.back();
}

const t_point_t & BSpline::controlPoints() const
{
  return controlPoints_;
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
  for(unsigned int i = 0; i < this->controlPoints().size() - 1; ++i)
  {
    gui.addElement(waypointCategory,
                   mc_rtc::gui::Point3D("Waypoint " + std::to_string(i),
                                        [this, i]() -> const Eigen::Vector3d & { return controlPoints_[i]; },
                                        [this, i](const Eigen::Vector3d & pos) {
                                          controlPoints_[i] = pos;
                                          this->controlPoints(controlPoints_);
                                        }));
  }
}

} // namespace mc_trajectory
