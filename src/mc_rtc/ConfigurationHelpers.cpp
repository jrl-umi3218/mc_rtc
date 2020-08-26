#include <mc_rtc/ConfigurationHelpers.h>

namespace mc_rtc
{

void overwriteRotationRPY(const mc_rtc::Configuration & c, Eigen::Matrix3d & rotation)
{
  if(c.has("roll") || c.has("pitch") || c.has("yaw"))
  {
    Eigen::Vector3d rpy = mc_rbdyn::rpyFromMat(rotation);
    c("roll", rpy.x());
    c("pitch", rpy.y());
    c("yaw", rpy.z());
    rotation = mc_rbdyn::rpyToMat(rpy);
  }
}

} // namespace mc_rtc
