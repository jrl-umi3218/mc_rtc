/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/mc_global_controller.h>
#include <mc_rtc/ros.h>

#ifdef MC_RTC_HAS_ROS
#  include <mc_rtc_msgs/EnableController.h>
#  include <mc_rtc_msgs/close_grippers.h>
#  include <mc_rtc_msgs/get_joint_pos.h>
#  include <mc_rtc_msgs/move_com.h>
#  include <mc_rtc_msgs/open_grippers.h>
#  include <mc_rtc_msgs/play_next_stance.h>
#  include <mc_rtc_msgs/send_msg.h>
#  include <mc_rtc_msgs/send_recv_msg.h>
#  include <mc_rtc_msgs/set_gripper.h>
#  include <mc_rtc_msgs/set_joint_pos.h>
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
  bool move_com_callback(mc_rtc_msgs::move_comRequest & req, mc_rtc_msgs::move_comResponse & res);
  bool EnableController_callback(mc_rtc_msgs::EnableController::Request & req,
                                 mc_rtc_msgs::EnableController::Response & resp);
  bool close_grippers_callback(mc_rtc_msgs::close_grippers::Request &, mc_rtc_msgs::close_grippers::Response & resp);
  bool open_grippers_callback(mc_rtc_msgs::open_grippers::Request &, mc_rtc_msgs::open_grippers::Response & resp);
  bool set_gripper_callback(mc_rtc_msgs::set_gripper::Request & req, mc_rtc_msgs::set_gripper::Response & resp);
  bool set_joint_pos_callback(mc_rtc_msgs::set_joint_pos::Request & req, mc_rtc_msgs::set_joint_pos::Response & resp);
  bool get_joint_pos_callback(mc_rtc_msgs::get_joint_posRequest & req, mc_rtc_msgs::get_joint_posResponse & res);
  bool play_next_stance_callback(mc_rtc_msgs::play_next_stance::Request &,
                                 mc_rtc_msgs::play_next_stance::Response & resp);
  bool send_msg_callback(mc_rtc_msgs::send_msg::Request & req, mc_rtc_msgs::send_msg::Response & resp);
  bool send_recv_msg_callback(mc_rtc_msgs::send_recv_msg::Request & req, mc_rtc_msgs::send_recv_msg::Response & resp);
#endif
};

} // namespace mc_control
