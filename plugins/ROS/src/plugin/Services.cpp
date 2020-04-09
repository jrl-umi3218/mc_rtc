/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "Services.h"

namespace mc_plugin
{

ROSServices::ROSServices(std::shared_ptr<ros::NodeHandle> nh, mc_control::MCGlobalController & controller)
: nh_(nh), controller_(controller)
{
  if(nh)
  {
    start_services();
  }
  else
  {
    LOG_WARNING("ROS not available, services will not be enabled")
  }
}

void ROSServices::start_services()
{
  LOG_SUCCESS("[mc_rtc::ROS] Starting ROS services")
  services.push_back(nh_->advertiseService("mc_rtc/enable_controller", &ROSServices::EnableController_callback, this));
  services.push_back(nh_->advertiseService("mc_rtc/close_grippers", &ROSServices::close_grippers_callback, this));
  services.push_back(nh_->advertiseService("mc_rtc/open_grippers", &ROSServices::open_grippers_callback, this));
  services.push_back(nh_->advertiseService("mc_rtc/set_gripper", &ROSServices::set_gripper_callback, this));
}

bool ROSServices::EnableController_callback(mc_rtc_msgs::EnableController::Request & req,
                                            mc_rtc_msgs::EnableController::Response & resp)
{
  LOG_INFO("[mc_rtc::ROS] Enable controller " << req.name);
  resp.success = controller_.EnableController(req.name);
  return true;
}

bool ROSServices::close_grippers_callback(mc_rtc_msgs::close_grippers::Request &,
                                          mc_rtc_msgs::close_grippers::Response & resp)
{
  LOG_INFO("[mc_rtc::ROS] close grippers");
  controller_.setGripperOpenPercent(controller_.robot().name(), 0.);
  resp.success = true;
  return true;
}

bool ROSServices::open_grippers_callback(mc_rtc_msgs::open_grippers::Request &,
                                         mc_rtc_msgs::open_grippers::Response & resp)
{
  LOG_INFO("[mc_rtc::ROS] Open grippers");
  controller_.setGripperOpenPercent(controller_.robot().name(), 1.);
  resp.success = true;
  return true;
}

bool ROSServices::set_gripper_callback(mc_rtc_msgs::set_gripper::Request & req,
                                       mc_rtc_msgs::set_gripper::Response & resp)
{
  LOG_INFO("[mc_rtc::ROS] Set gripper " << req.gname);
  controller_.setGripperTargetQ(controller_.robot().name(), req.gname, req.values);
  resp.success = true;
  return true;
}

} // namespace mc_plugin
