/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_trajectory/BSplineTrajectory.h>

namespace mc_trajectory
{

BSplineTrajectory::BSplineTrajectory(const std::vector<Eigen::Vector3d> & controlPoints,
                                     double duration,
                                     unsigned int order)
: duration(duration), p(order), P(3, 0), knot(0, 1), spline(knot, P)
{
  P.resize(3, 2 * (static_cast<int>(order) - 2) + static_cast<int>(controlPoints.size()));
  for(unsigned int i = 0; i < order - 2; ++i)
  {
    P(0, i) = controlPoints[0](0);
    P(1, i) = controlPoints[0](1);
    P(2, i) = controlPoints[0](2);
  }
  for(unsigned int i = 0; i < controlPoints.size(); ++i)
  {
    P(0, i + order - 2) = controlPoints[i](0);
    P(1, i + order - 2) = controlPoints[i](1);
    P(2, i + order - 2) = controlPoints[i](2);
  }
  for(unsigned int i = 0; i < order - 2; ++i)
  {
    P(0, i + order - 2 + static_cast<unsigned int>(controlPoints.size())) = controlPoints[controlPoints.size() - 1](0);
    P(1, i + order - 2 + static_cast<unsigned int>(controlPoints.size())) = controlPoints[controlPoints.size() - 1](1);
    P(2, i + order - 2 + static_cast<unsigned int>(controlPoints.size())) = controlPoints[controlPoints.size() - 1](2);
  }
  unsigned int n = static_cast<unsigned int>(P.cols()) - 1;
  unsigned int m = p + n + 1;

  knot.resize(m + 1);
  for(unsigned int i = 0; i < p; ++i)
  {
    knot(i) = 0;
  }
  for(unsigned int i = 0; i < (m + 1 - 2 * p); ++i)
  {
    knot(i + p) = static_cast<double>(i) / static_cast<double>((m - 2 * p));
  }
  for(unsigned int i = 0; i < p; ++i)
  {
    knot(m + 1 - p + i) = 1.0;
  }
  spline = Spline3d(knot, P);
}

std::vector<std::vector<Eigen::Vector3d>> BSplineTrajectory::splev(const std::vector<double> & t, unsigned int der)
{
  std::vector<std::vector<Eigen::Vector3d>> res(0);
  res.reserve(t.size());
  for(const auto & ti : t)
  {
    std::vector<Eigen::Vector3d> pts;
    pts.reserve(der + 1);
    auto tmp = spline.derivatives(ti / duration, der);
    for(unsigned int i = 0; i <= der; ++i)
    {
      Eigen::Vector3d cu;
      cu(0) = tmp(0, i);
      cu(1) = tmp(1, i);
      cu(2) = tmp(2, i);
      pts.push_back(cu);
    }
    res.push_back(pts);
  }
  return res;
}

} // namespace mc_trajectory
