/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL, BIT
 */

#pragma once

#include <mc_control/mc_global_controller.h>
#include <mc_rtc_ros/ros.h>

#ifdef MC_RTC_ROS_IS_ROS2
#  include <mc_rtc_msgs/srv/close_grippers.hpp>
#  include <mc_rtc_msgs/srv/enable_controller.hpp>
#  include <mc_rtc_msgs/srv/open_grippers.hpp>
#  include <mc_rtc_msgs/srv/set_gripper.hpp>
#  include <rclcpp/rclcpp.hpp>
#else
#  include <mc_rtc_msgs/EnableController.h>
#  include <mc_rtc_msgs/close_grippers.h>
#  include <mc_rtc_msgs/open_grippers.h>
#  include <mc_rtc_msgs/set_gripper.h>
#  include <ros/ros.h>
#endif

namespace mc_plugin
{

struct MC_CONTROL_DLLAPI ROSServices
{
  ROSServices(mc_rtc::NodeHandlePtr nh, mc_control::MCGlobalController & controller);

private:
  void start_services();
  mc_rtc::NodeHandlePtr nh_;
  mc_control::MCGlobalController & controller_;
#ifdef MC_RTC_ROS_IS_ROS2
  rclcpp::Service<mc_rtc_msgs::srv::EnableController>::SharedPtr enable_ctl_service;
  rclcpp::Service<mc_rtc_msgs::srv::CloseGrippers>::SharedPtr close_grippers_service;
  rclcpp::Service<mc_rtc_msgs::srv::OpenGrippers>::SharedPtr open_grippers_service;
  rclcpp::Service<mc_rtc_msgs::srv::SetGripper>::SharedPtr set_gripper_service;

  void EnableController_callback(const mc_rtc_msgs::srv::EnableController::Request & req,
                                 mc_rtc_msgs::srv::EnableController::Response & resp);
  void close_grippers_callback(const mc_rtc_msgs::srv::CloseGrippers::Request &,
                               mc_rtc_msgs::srv::CloseGrippers::Response & resp);
  void open_grippers_callback(const mc_rtc_msgs::srv::OpenGrippers::Request &,
                              mc_rtc_msgs::srv::OpenGrippers::Response & resp);
  void set_gripper_callback(const mc_rtc_msgs::srv::SetGripper::Request &,
                            mc_rtc_msgs::srv::SetGripper::Response & resp);
#else
  std::vector<ros::ServiceServer> services;
  bool EnableController_callback(mc_rtc_msgs::EnableController::Request & req,
                                 mc_rtc_msgs::EnableController::Response & resp);
  bool close_grippers_callback(mc_rtc_msgs::close_grippers::Request &, mc_rtc_msgs::close_grippers::Response & resp);
  bool open_grippers_callback(mc_rtc_msgs::open_grippers::Request &, mc_rtc_msgs::open_grippers::Response & resp);
  bool set_gripper_callback(mc_rtc_msgs::set_gripper::Request & req, mc_rtc_msgs::set_gripper::Response & resp);
#endif
};

} // namespace mc_plugin
