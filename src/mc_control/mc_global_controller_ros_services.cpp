/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_global_controller_ros_services.h"

namespace mc_control
{

ROSServicesImpl::ROSServicesImpl(std::shared_ptr<ros::NodeHandle> nh, mc_control::MCGlobalController & controller)
: nh_(nh), controller_(controller)
{
  if(nh)
  {
    start_services();
  }
  else
  {
#ifdef MC_RTC_HAS_ROS
    LOG_WARNING("ROS not available, services will not be enabled")
#endif
  }
}

void ROSServicesImpl::start_services()
{
#ifdef MC_RTC_HAS_ROS
  LOG_SUCCESS("[MCGlobalControllerServices] Starting ROS services")
  services.push_back(
      nh_->advertiseService("mc_rtc/enable_controller", &ROSServicesImpl::EnableController_callback, this));
  services.push_back(nh_->advertiseService("mc_rtc/close_grippers", &ROSServicesImpl::close_grippers_callback, this));
  services.push_back(nh_->advertiseService("mc_rtc/open_grippers", &ROSServicesImpl::open_grippers_callback, this));
  services.push_back(nh_->advertiseService("mc_rtc/set_gripper", &ROSServicesImpl::set_gripper_callback, this));
  services.push_back(nh_->advertiseService("mc_rtc/set_joint_pos", &ROSServicesImpl::set_joint_pos_callback, this));
  services.push_back(nh_->advertiseService("mc_rtc/get_joint_pos", &ROSServicesImpl::get_joint_pos_callback, this));
  services.push_back(
      nh_->advertiseService("mc_rtc/play_next_stance", &ROSServicesImpl::play_next_stance_callback, this));
  services.push_back(nh_->advertiseService("mc_rtc/send_msg", &ROSServicesImpl::send_msg_callback, this));
  services.push_back(nh_->advertiseService("mc_rtc/send_recv_msg", &ROSServicesImpl::send_recv_msg_callback, this));
  services.push_back(nh_->advertiseService("mc_rtc/move_com", &ROSServicesImpl::move_com_callback, this));
#endif
}

#ifdef MC_RTC_HAS_ROS
bool ROSServicesImpl::move_com_callback(mc_rtc_msgs::move_comRequest & req, mc_rtc_msgs::move_comResponse & res)
{
  LOG_INFO("[MCGlobalControllerServices] Moving CoM to (" << req.com[0] << ", " << req.com[1] << ", " << req.com[2]
                                                          << ")");
  res.success = controller_.move_com({req.com[0], req.com[1], req.com[2]});
  return res.success;
}

bool ROSServicesImpl::EnableController_callback(mc_rtc_msgs::EnableController::Request & req,
                                                mc_rtc_msgs::EnableController::Response & resp)
{
  LOG_INFO("[MCGlobalControllerServices] Enable controller " << req.name);
  resp.success = controller_.EnableController(req.name);
  return true;
}

bool ROSServicesImpl::close_grippers_callback(mc_rtc_msgs::close_grippers::Request &,
                                              mc_rtc_msgs::close_grippers::Response & resp)
{
  LOG_INFO("[MCGlobalControllerServices] close grippers");
  controller_.setGripperOpenPercent(0.);
  resp.success = true;
  return true;
}

bool ROSServicesImpl::open_grippers_callback(mc_rtc_msgs::open_grippers::Request &,
                                             mc_rtc_msgs::open_grippers::Response & resp)
{
  LOG_INFO("[MCGlobalControllerServices] Open grippers");
  controller_.setGripperOpenPercent(1.);
  resp.success = true;
  return true;
}

bool ROSServicesImpl::set_gripper_callback(mc_rtc_msgs::set_gripper::Request & req,
                                           mc_rtc_msgs::set_gripper::Response & resp)
{
  LOG_INFO("[MCGlobalControllerServices] Set gripper " << req.gname);
  controller_.setGripperTargetQ(req.gname, req.values);
  resp.success = true;
  return true;
}

bool ROSServicesImpl::set_joint_pos_callback(mc_rtc_msgs::set_joint_pos::Request & req,
                                             mc_rtc_msgs::set_joint_pos::Response & resp)
{
  LOG_INFO("[MCGlobalControllerServices] Setting joint pos " << req.jname << " = " << req.q);
  resp.success = controller_.set_joint_pos(req.jname, req.q);
  return true;
}

bool ROSServicesImpl::get_joint_pos_callback(mc_rtc_msgs::get_joint_posRequest & req,
                                             mc_rtc_msgs::get_joint_posResponse & res)
{
  res.success = controller_.get_joint_pos(req.jname, res.q);
  LOG_INFO("[MCGlobalControllerServices] Joint pos " << req.jname << " = " << res.q);
  return res.success;
}

bool ROSServicesImpl::play_next_stance_callback(mc_rtc_msgs::play_next_stance::Request &,
                                                mc_rtc_msgs::play_next_stance::Response & resp)
{
  LOG_INFO("[MCGlobalControllerServices] Playing next stance");
  resp.success = controller_.play_next_stance();
  return true;
}

bool ROSServicesImpl::send_msg_callback(mc_rtc_msgs::send_msg::Request & req, mc_rtc_msgs::send_msg::Response & resp)
{
  LOG_INFO("[MCGlobalControllerServices] Sending message " << req.msg);
  resp.success = controller_.send_msg(req.msg);
  return true;
}

bool ROSServicesImpl::send_recv_msg_callback(mc_rtc_msgs::send_recv_msg::Request & req,
                                             mc_rtc_msgs::send_recv_msg::Response & resp)
{
  LOG_INFO("[MCGlobalControllerServices] Sending message " << req.msg);
  resp.success = controller_.send_recv_msg(req.msg, resp.msg);
  LOG_INFO("Received message: " << resp.msg);
  return true;
}
#endif // MC_RTC_HAS_ROS

} // namespace mc_control
