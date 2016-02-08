#pragma once

#include <memory>
#include <vector>

#include <mc_rtc/config.h>
#ifdef MC_RTC_HAS_HRPSYS_BASE
#include <rtm/idl/ExtendedDataTypesSkel.h>
#else
namespace RTC
{
  struct TimedPoint3D {};
  struct TimedOrientation3D {};
  struct TimedAcceleration3D {};
  struct TimedAngularVelocity3D {};
}
#endif

#include <mc_rtc/ros_api.h>

namespace ros
{
  class NodeHandle;
}

namespace mc_rbdyn
{
  struct Robot;
}

namespace mc_rtc
{

struct ROSBridgeImpl;

struct MC_RTC_ROS_DLLAPI ROSBridge
{
  static std::shared_ptr<ros::NodeHandle> get_node_handle();

  static void update_robot_publisher(double dt, const mc_rbdyn::Robot & robot, const RTC::TimedPoint3D & p, const RTC::TimedOrientation3D & rpy, const RTC::TimedAngularVelocity3D & rate, const RTC::TimedAcceleration3D & gsensor, const std::vector<double> & lGq = {}, const std::vector<double> & rGq = {});

  static void reset_imu_offset();

  static void shutdown();
private:
  static std::unique_ptr<ROSBridgeImpl> impl;
};

}
