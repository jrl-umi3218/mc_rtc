/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_trajectory/BSpline.h>

#include <iostream>

int main()
{
  using waypoints_t = mc_trajectory::BSpline::waypoints_t;
  Eigen::Vector3d start{-0.45, -0.34, 0.54};
  Eigen::Vector3d target{0.45, -0.34, 0.54};
  waypoints_t cps;
  cps.push_back(Eigen::Vector3d(0.4, -0.34, 0.54));
  mc_trajectory::BSpline spline(20.0, start, target, cps);
  for(double t = 0; t < 20.01; t += 1.0)
  {
    // auto res = spline.spline(t/20.0);
    // std::cout << "At t = " << t << std::endl;
    // std::cout << res(0) << std::endl;
    auto res = spline.splev(t, 2);
    auto & pos = res[0];
    auto & vel = res[1];
    auto & acc = res[2];
    std::cout << "At t = " << t << std::endl;
    std::cout << "pos(0)" << pos(0) << std::endl;
    std::cout << "vel" << std::endl << vel << std::endl;
    std::cout << "acc" << std::endl << acc << std::endl;
  }
  return 0;
}
