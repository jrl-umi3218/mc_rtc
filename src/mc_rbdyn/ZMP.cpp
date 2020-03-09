#include <mc_rbdyn/ZMP.h>
#include <mc_rtc/logging.h>
#include <stdexcept>

namespace mc_rbdyn
{

Eigen::Vector3d zmp(const sva::ForceVecd & netTotalWrench,
                    const Eigen::Vector3d & plane_p,
                    const Eigen::Vector3d & plane_n,
                    double minimalNetNormalForce)
{
  if(minimalNetNormalForce <= 0)
  {
    LOG_ERROR_AND_THROW(std::runtime_error,
                        "ZMP cannot be computed: the minimalNetNormalForce must be >0 (divide by zero)");
  }

  const Eigen::Vector3d & force = netTotalWrench.force();
  const Eigen::Vector3d & moment_0 = netTotalWrench.couple();
  Eigen::Vector3d moment_p = moment_0 - plane_p.cross(force);
  double floorn_dot_force = plane_n.dot(force);
  // Prevent potential division by zero
  if(floorn_dot_force < minimalNetNormalForce)
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "ZMP cannot be computed, projected force too small " << floorn_dot_force);
  }
  Eigen::Vector3d zmp = plane_p + plane_n.cross(moment_p) / floorn_dot_force;
  return zmp;
}

Eigen::Vector3d zmp(const sva::ForceVecd & netWrench, const sva::PTransformd & zmpFrame, double minimalNetNormalForce)
{
  Eigen::Vector3d n = zmpFrame.rotation().row(2);
  Eigen::Vector3d p = zmpFrame.translation();
  return zmp(netWrench, p, n, minimalNetNormalForce);
}

} // namespace mc_rbdyn
