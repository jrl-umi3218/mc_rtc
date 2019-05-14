/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_trajectory/BSplineTrajectory.h>

namespace mc_trajectory
{

BSplineTrajectory::BSplineTrajectory(const std::vector<point_t> & controlPoints,
                                                           double duration,
                                                           unsigned int order)
: duration(duration), p(order), spline(controlPoints.begin(), controlPoints.end(), duration)
{
}

std::vector<std::vector<point_t>> BSplineTrajectory::splev(const std::vector<double> & t, unsigned int der)
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

std::vector<Eigen::Vector3d> BSplineTrajectory::sampleTrajectory(unsigned samples)
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

void BSplineTrajectory::controlPoints(const t_point_t& waypoints)
{
  spline = bezier_curve_t(waypoints.begin(), waypoints.end(), duration);
}

const t_point_t& BSplineTrajectory::controlPoints() const
{
  return spline.waypoints();
}


void BSplineTrajectory::addToGUI(mc_rtc::gui::StateBuilder & gui, const std::vector<std::string>& category)
{
  // Visual controls for the control points and
  for(unsigned int i = 0; i < this->controlPoints().size(); ++i)
  {
    gui.addElement(category,
                   mc_rtc::gui::Point3D("control_point_pos_" + std::to_string(i),
                                        [this, i]() { return this->controlPoints()[i]; },
                                        [this, i](const Eigen::Vector3d & pos) {
                                          auto waypoints = this->controlPoints();
                                          waypoints[i] = pos;
                                          this->controlPoints(waypoints);
                                          this->needsUpdate_ = true;
                                        }));

  }

  samples_ = this->sampleTrajectory(samplingPoints_);
  gui.addElement(category, mc_rtc::gui::Trajectory("trajectory",
                [this]() {
                    if(this->needsUpdate_)
                    {
                      samples_ = this->sampleTrajectory(samplingPoints_);
                      needsUpdate_ = false;
                    }
                    return samples_;
                }));
}


} // namespace mc_trajectory
