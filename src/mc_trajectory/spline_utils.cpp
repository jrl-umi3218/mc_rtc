#include <mc_trajectory/spline_utils.h>

namespace mc_trajectory
{

Eigen::MatrixXd generateInterpolatedWaypoints(const Eigen::Vector3d & start,
                                              const Eigen::Vector3d & stop,
                                              unsigned int nrWP)
{
  Eigen::MatrixXd res(3, nrWP);
  for(unsigned int i = 0; i < nrWP; ++i)
  {
    res(0, i) = start(0) + (stop(0) - start(0)) * (static_cast<double>(i) + 1.0) / (static_cast<double>(nrWP) + 1.0);
    res(1, i) = start(1) + (stop(1) - start(1)) * (static_cast<double>(i) + 1.0) / (static_cast<double>(nrWP) + 1.0);
    res(2, i) = start(2) + (stop(2) - start(2)) * (static_cast<double>(i) + 1.0) / (static_cast<double>(nrWP) + 1.0);
  }
  return res;
}

} // namespace mc_trajectory
