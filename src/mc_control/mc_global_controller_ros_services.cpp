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
#endif
}

#ifdef MC_RTC_HAS_ROS
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
#endif // MC_RTC_HAS_ROS

} // namespace mc_control
