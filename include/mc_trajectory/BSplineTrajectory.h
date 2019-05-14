/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifndef _H_BSPLINETRAJECTORY_H_
#define _H_BSPLINETRAJECTORY_H_

#include <mc_rtc/GUIState.h>
#include <mc_trajectory/api.h>

#include <hpp/spline/bezier_curve.h>
#include <vector>

namespace mc_trajectory
{

using point_t = Eigen::Vector3d;
using time_t = double;
using bezier_curve_t = spline::bezier_curve<time_t, double, 3, true, point_t>;
using t_point_t = bezier_curve_t::t_point_t;

struct MC_TRAJECTORY_DLLAPI BSplineTrajectory
{
public:
  BSplineTrajectory(const std::vector<point_t> & controlPoints, double duration, unsigned int order = 4);

  std::vector<std::vector<Eigen::Vector3d>> splev(const std::vector<double> & t, unsigned int der = 0);
  std::vector<Eigen::Vector3d> sampleTrajectory(unsigned samples);

  void controlPoints(const t_point_t & waypoints);
  const t_point_t & controlPoints() const;

  void addToGUI(mc_rtc::gui::StateBuilder & gui, const std::vector<std::string> & category);

private:
  double duration;
  unsigned int p;
  bezier_curve_t spline;
  bool needsUpdate_ = false;
  size_t samplingPoints_ = 10;
  std::vector<point_t> samples_;
};

} // namespace mc_trajectory

#endif
