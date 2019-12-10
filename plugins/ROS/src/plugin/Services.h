/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL, BIT
 */

#pragma once

#include <mc_control/mc_global_controller.h>
#include <mc_rtc/ros.h>

#include <mc_rtc_msgs/EnableController.h>
#include <mc_rtc_msgs/close_grippers.h>
#include <mc_rtc_msgs/open_grippers.h>
#include <mc_rtc_msgs/set_gripper.h>
#include <ros/ros.h>

namespace mc_plugin
{

struct MC_CONTROL_DLLAPI ROSServices
{
  ROSServices(std::shared_ptr<ros::NodeHandle> nh, mc_control::MCGlobalController & controller);

private:
  void start_services();
  std::shared_ptr<ros::NodeHandle> nh_;
  mc_control::MCGlobalController & controller_;
  std::vector<ros::ServiceServer> services;
  bool EnableController_callback(mc_rtc_msgs::EnableController::Request & req,
                                 mc_rtc_msgs::EnableController::Response & resp);
  bool close_grippers_callback(mc_rtc_msgs::close_grippers::Request &, mc_rtc_msgs::close_grippers::Response & resp);
  bool open_grippers_callback(mc_rtc_msgs::open_grippers::Request &, mc_rtc_msgs::open_grippers::Response & resp);
  bool set_gripper_callback(mc_rtc_msgs::set_gripper::Request & req, mc_rtc_msgs::set_gripper::Response & resp);
};

} // namespace mc_plugin
