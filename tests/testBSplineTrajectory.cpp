#include <mc_trajectory/BSplineTrajectory.h>

#include <iostream>

int main()
{
  std::vector<Eigen::Vector3d> cps;
  cps.push_back(Eigen::Vector3d(-0.45, -0.34, 0.54));
  cps.push_back(Eigen::Vector3d(0.4, -0.34, 0.54));
  cps.push_back(Eigen::Vector3d(0.45, -0.34, 0.54));
  mc_trajectory::BSplineTrajectory spline(cps, 20.0);
  for(double t = 0; t < 20.01; t += 1.0)
  {
    // auto res = spline.spline(t/20.0);
    // std::cout << "At t = " << t << std::endl;
    // std::cout << res(0) << std::endl;
    auto res = spline.splev({t}, 2);
    auto & pos = res[0][0];
    auto & vel = res[0][1];
    auto & acc = res[0][2];
    std::cout << "At t = " << t << std::endl;
    std::cout << "pos(0)" << pos(0) << std::endl;
    std::cout << "vel" << std::endl << vel << std::endl;
    std::cout << "acc" << std::endl << acc << std::endl;
  }
  return 0;
}
