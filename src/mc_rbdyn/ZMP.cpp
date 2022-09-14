#include <mc_rbdyn/ZMP.h>
#include <mc_rtc/logging.h>
#include <stdexcept>

namespace mc_rbdyn
{

bool zmp(Eigen::Vector3d & zmpOut,
         const sva::ForceVecd & netTotalWrench,
         const Eigen::Vector3d & plane_p,
         const Eigen::Vector3d & plane_n,
         double minimalNetNormalForce) noexcept
{
  assert(minimalNetNormalForce > 0);
  const Eigen::Vector3d & force = netTotalWrench.force();
  const Eigen::Vector3d & moment_0 = netTotalWrench.couple();
  Eigen::Vector3d moment_p = moment_0 - plane_p.cross(force);
  double floorn_dot_force = plane_n.dot(force);
  // Prevent potential division by zero
  if(floorn_dot_force < minimalNetNormalForce)
  {
    mc_rtc::log::error("ZMP cannot be computed, projected force too small {}", floorn_dot_force);
    return false;
  }
  zmpOut = plane_p + plane_n.cross(moment_p) / floorn_dot_force;
  return true;
}

Eigen::Vector3d zmp(const sva::ForceVecd & netTotalWrench,
                    const Eigen::Vector3d & plane_p,
                    const Eigen::Vector3d & plane_n,
                    double minimalNetNormalForce)
{
  if(minimalNetNormalForce <= 0)
  {
    mc_rtc::log::error_and_throw("ZMP cannot be computed: the minimalNetNormalForce must be >0 (divide by zero)");
  }

  Eigen::Vector3d zmpOut;
  if(!zmp(zmpOut, netTotalWrench, plane_p, plane_n, minimalNetNormalForce))
  {
    mc_rtc::log::error_and_throw("ZMP cannot be computed");
  }
  return zmpOut;
}

Eigen::Vector3d zmp(const sva::ForceVecd & netWrench, const sva::PTransformd & zmpFrame, double minimalNetNormalForce)
{
  Eigen::Vector3d n = zmpFrame.rotation().row(2);
  Eigen::Vector3d p = zmpFrame.translation();
  return zmp(netWrench, p, n, minimalNetNormalForce);
}

bool zmp(Eigen::Vector3d & zmpOut,
         const sva::ForceVecd & netWrench,
         const sva::PTransformd & zmpFrame,
         double minimalNetNormalForce) noexcept
{
  Eigen::Vector3d n = zmpFrame.rotation().row(2);
  Eigen::Vector3d p = zmpFrame.translation();
  return zmp(zmpOut, netWrench, p, n, minimalNetNormalForce);
}

} // namespace mc_rbdyn
