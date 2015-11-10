#pragma once

#include <memory>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <vector>

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

struct ROSBridge
{
  static std::shared_ptr<ros::NodeHandle> get_node_handle();

  static void update_robot_publisher(const mc_rbdyn::Robot & robot, const RTC::TimedPoint3D & p, const RTC::TimedOrientation3D & rpy, const RTC::TimedAcceleration3D & gsensor, const std::vector<double> & lGq = {}, const std::vector<double> & rGq = {});

  static void shutdown();
private:
  static std::unique_ptr<ROSBridgeImpl> impl;
};

}
