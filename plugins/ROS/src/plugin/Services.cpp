/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "Services.h"

namespace mc_plugin
{

ROSServices::ROSServices(mc_rtc::NodeHandlePtr nh, mc_control::MCGlobalController & controller)
: nh_(nh), controller_(controller)
{
  if(nh) { start_services(); }
  else
  {
    mc_rtc::log::warning("ROS not available, services will not be enabled");
  }
}

void ROSServices::start_services()
{
  mc_rtc::log::success("[mc_rtc::ROS] Starting ROS services");
  enable_ctl_service = nh_->create_service<mc_rtc_msgs::srv::EnableController>(
      "mc_rtc/enable_controller", [this](const std::shared_ptr<mc_rtc_msgs::srv::EnableController::Request> req,
                                         std::shared_ptr<mc_rtc_msgs::srv::EnableController::Response> resp)
      { EnableController_callback(*req, *resp); });
  close_grippers_service = nh_->create_service<mc_rtc_msgs::srv::CloseGrippers>(
      "mc_rtc/close_grippers", [this](const std::shared_ptr<mc_rtc_msgs::srv::CloseGrippers::Request> req,
                                      std::shared_ptr<mc_rtc_msgs::srv::CloseGrippers::Response> resp)
      { close_grippers_callback(*req, *resp); });
  open_grippers_service = nh_->create_service<mc_rtc_msgs::srv::OpenGrippers>(
      "mc_rtc/open_grippers",
      [this](const std::shared_ptr<mc_rtc_msgs::srv::OpenGrippers::Request> req,
             std::shared_ptr<mc_rtc_msgs::srv::OpenGrippers::Response> resp) { open_grippers_callback(*req, *resp); });
  set_gripper_service = nh_->create_service<mc_rtc_msgs::srv::SetGripper>(
      "mc_rtc/set_gripper",
      [this](const std::shared_ptr<mc_rtc_msgs::srv::SetGripper::Request> req,
             std::shared_ptr<mc_rtc_msgs::srv::SetGripper::Response> resp) { set_gripper_callback(*req, *resp); });
}

void ROSServices::EnableController_callback(const mc_rtc_msgs::srv::EnableController::Request & req,
                                            mc_rtc_msgs::srv::EnableController::Response & resp)
{
  mc_rtc::log::info("[mc_rtc::ROS] Enable controller {}", req.name);
  resp.success = controller_.EnableController(req.name);
}

void ROSServices::close_grippers_callback(const mc_rtc_msgs::srv::CloseGrippers::Request &,
                                          mc_rtc_msgs::srv::CloseGrippers::Response & resp)
{
  mc_rtc::log::info("[mc_rtc::ROS] close grippers");
  controller_.setGripperOpenPercent(controller_.robot().name(), 0.);
  resp.success = true;
}

void ROSServices::open_grippers_callback(const mc_rtc_msgs::srv::OpenGrippers::Request &,
                                         mc_rtc_msgs::srv::OpenGrippers::Response & resp)
{
  mc_rtc::log::info("[mc_rtc::ROS] Open grippers");
  controller_.setGripperOpenPercent(controller_.robot().name(), 1.);
  resp.success = true;
}

void ROSServices::set_gripper_callback(const mc_rtc_msgs::srv::SetGripper::Request & req,
                                       mc_rtc_msgs::srv::SetGripper::Response & resp)
{
  mc_rtc::log::info("[mc_rtc::ROS] Set gripper {}", req.gname);
  controller_.setGripperTargetQ(controller_.robot().name(), req.gname, req.values);
  resp.success = true;
}

} // namespace mc_plugin
