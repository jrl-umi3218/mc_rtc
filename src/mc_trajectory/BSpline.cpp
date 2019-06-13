/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_trajectory/BSpline.h>

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

std::vector<std::vector<Eigen::Vector3d>> BSpline::splev(const std::vector<double> & t, unsigned int der)
{
  std::vector<std::vector<point_t>> res(0);
  res.reserve(t.size());
  for(const auto & ti : t)
  {
    std::vector<Eigen::Vector3d> pts;
    pts.reserve(der + 1);
    for(std::size_t order = 0; order <= der; ++order)
    {
      pts.push_back(spline->derivate(ti, order));
    }
    res.push_back(pts);
  }
  return res;
}

std::vector<Eigen::Vector3d> BSpline::sampleTrajectory(unsigned samples)
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

void BSpline::controlPoints(const t_point_t & waypoints)
{
  spline.reset(new bezier_curve_t(waypoints.begin(), waypoints.end(), duration));
  needsUpdate_ = true;
}

void BSpline::target(const point_t & target)
{
  controlPoints_.back() = target;
  this->controlPoints(controlPoints_);
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
  // Visual controls for the control points and
  for(unsigned int i = 0; i < this->controlPoints().size() - 1; ++i)
  {
    gui.addElement(category, mc_rtc::gui::Point3D("control_point_pos_" + std::to_string(i),
                                                  [this, i]() { return controlPoints_[i]; },
                                                  [this, i](const Eigen::Vector3d & pos) {
                                                    controlPoints_[i] = pos;
                                                    this->controlPoints(controlPoints_);
                                                  }));
  }

  samples_ = this->sampleTrajectory(samplingPoints_);
  gui.addElement(category, mc_rtc::gui::Trajectory("trajectory", [this]() {
                   if(this->needsUpdate_)
                   {
                     samples_ = this->sampleTrajectory(samplingPoints_);
                     needsUpdate_ = false;
                   }
                   return samples_;
                 }));
}

} // namespace mc_trajectory
