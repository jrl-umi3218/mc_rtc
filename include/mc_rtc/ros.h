#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <mc_rtc/config.h>

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

  static void update_robot_publisher(double dt, const mc_rbdyn::Robot & robot, const Eigen::Vector3d & p, const Eigen::Vector3d & rpy, const Eigen::Vector3d & rate, const Eigen::Vector3d & gsensor, const std::map<std::string, std::vector<std::string>> & gripperJ, const std::map<std::string, std::vector<double>> & gripperQ);

  static void reset_imu_offset();

  static void shutdown();
private:
  static std::unique_ptr<ROSBridgeImpl> impl;
};

}
