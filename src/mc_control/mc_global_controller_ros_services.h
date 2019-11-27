/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/mc_global_controller.h>
#include <mc_rtc/ros.h>

#ifdef MC_RTC_HAS_ROS
#  include <mc_rtc_msgs/EnableController.h>
#  include <mc_rtc_msgs/close_grippers.h>
#  include <mc_rtc_msgs/open_grippers.h>
#  include <mc_rtc_msgs/set_gripper.h>
#  include <ros/ros.h>
#endif

namespace mc_control
{

struct MC_CONTROL_DLLAPI ROSServicesImpl
{
  ROSServicesImpl(std::shared_ptr<ros::NodeHandle> nh, mc_control::MCGlobalController & controller);

private:
  void start_services();
  std::shared_ptr<ros::NodeHandle> nh_;
  mc_control::MCGlobalController & controller_;
#ifdef MC_RTC_HAS_ROS
  std::vector<ros::ServiceServer> services;
  bool EnableController_callback(mc_rtc_msgs::EnableController::Request & req,
                                 mc_rtc_msgs::EnableController::Response & resp);
  bool close_grippers_callback(mc_rtc_msgs::close_grippers::Request &, mc_rtc_msgs::close_grippers::Response & resp);
  bool open_grippers_callback(mc_rtc_msgs::open_grippers::Request &, mc_rtc_msgs::open_grippers::Response & resp);
  bool set_gripper_callback(mc_rtc_msgs::set_gripper::Request & req, mc_rtc_msgs::set_gripper::Response & resp);
#endif
};

} // namespace mc_control
