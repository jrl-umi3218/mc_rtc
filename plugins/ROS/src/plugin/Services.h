/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL, BIT
 */

#pragma once

#include <mc_control/mc_global_controller.h>
#include <mc_rtc_ros/ros.h>

#include <mc_rtc_msgs/srv/close_grippers.hpp>
#include <mc_rtc_msgs/srv/enable_controller.hpp>
#include <mc_rtc_msgs/srv/open_grippers.hpp>
#include <mc_rtc_msgs/srv/set_gripper.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mc_plugin
{

struct MC_CONTROL_DLLAPI ROSServices
{
  ROSServices(mc_rtc::NodeHandlePtr nh, mc_control::MCGlobalController & controller);

private:
  void start_services();
  mc_rtc::NodeHandlePtr nh_;
  mc_control::MCGlobalController & controller_;
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
};

} // namespace mc_plugin
