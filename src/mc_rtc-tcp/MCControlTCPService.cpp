#include "MCControlTCPService.h"

#include <mc_rtc/ros.h>

// ROS includes
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <ros/ros.h>
#include <mc_tcp_msgs/EnableController.h>
#include <mc_tcp_msgs/close_grippers.h>
#include <mc_tcp_msgs/open_grippers.h>
#include <mc_tcp_msgs/set_gripper.h>
#include <mc_tcp_msgs/set_joint_pos.h>
#include <mc_tcp_msgs/play_next_stance.h>
#include <mc_tcp_msgs/send_msg.h>
#include <mc_tcp_msgs/send_recv_msg.h>
#pragma GCC diagnostic pop

struct MCControlTCPServiceImpl
{
  MCControlTCPServiceImpl(mc_control::MCGlobalController & controller) :
    controller(controller),
    nh(mc_rtc::ROSBridge::get_node_handle())
  {
    if(nh)
    {
      LOG_SUCCESS("Starting ROS services")
      start_services();
    }
    else
    {
      LOG_WARNING("[MCControlTCP] ROS not available, services will not be enabled")
    }
  }
private:
  void start_services()
  {
    services.push_back(nh->advertiseService("mc_rtc/EnableController", &MCControlTCPServiceImpl::EnableController_callback, this));
    services.push_back(nh->advertiseService("mc_rtc/close_grippers", &MCControlTCPServiceImpl::close_grippers_callback, this));
    services.push_back(nh->advertiseService("mc_rtc/open_grippers", &MCControlTCPServiceImpl::open_grippers_callback, this));
    services.push_back(nh->advertiseService("mc_rtc/set_gripper", &MCControlTCPServiceImpl::set_gripper_callback, this));
    services.push_back(nh->advertiseService("mc_rtc/set_joint_pos", &MCControlTCPServiceImpl::set_joint_pos_callback, this));
    services.push_back(nh->advertiseService("mc_rtc/play_next_stance", &MCControlTCPServiceImpl::play_next_stance_callback, this));
    services.push_back(nh->advertiseService("mc_rtc/send_msg", &MCControlTCPServiceImpl::send_msg_callback, this));
    services.push_back(nh->advertiseService("mc_rtc/send_recv_msg", &MCControlTCPServiceImpl::send_recv_msg_callback, this));
  }

  bool EnableController_callback(mc_tcp_msgs::EnableController::Request & req, mc_tcp_msgs::EnableController::Response & resp)
  {
    resp.success = controller.EnableController(req.name);
    return true;
  }

  bool close_grippers_callback(mc_tcp_msgs::close_grippers::Request &, mc_tcp_msgs::close_grippers::Response & resp)
  {
    controller.setGripperOpenPercent(0.);
    resp.success = true;
    return true;
  }

  bool open_grippers_callback(mc_tcp_msgs::open_grippers::Request &, mc_tcp_msgs::open_grippers::Response & resp)
  {
    controller.setGripperOpenPercent(1.);
    resp.success = true;
    return true;
  }

  bool set_gripper_callback(mc_tcp_msgs::set_gripper::Request & req, mc_tcp_msgs::set_gripper::Response & resp)
  {
    controller.setGripperTargetQ(req.gname, req.values);
    resp.success = true;
    return true;
  }

  bool set_joint_pos_callback(mc_tcp_msgs::set_joint_pos::Request & req, mc_tcp_msgs::set_joint_pos::Response & resp)
  {
    resp.success = controller.set_joint_pos(req.jname, req.q);
    return true;
  }

  bool play_next_stance_callback(mc_tcp_msgs::play_next_stance::Request &, mc_tcp_msgs::play_next_stance::Response & resp)
  {
    resp.success = controller.play_next_stance();
    return true;
  }

  bool send_msg_callback(mc_tcp_msgs::send_msg::Request & req, mc_tcp_msgs::send_msg::Response & resp)
  {
    resp.success = controller.send_msg(req.msg);
    return true;
  }

  bool send_recv_msg_callback(mc_tcp_msgs::send_recv_msg::Request & req, mc_tcp_msgs::send_recv_msg::Response & resp)
  {
    resp.success = controller.send_recv_msg(req.msg, resp.msg);
    return true;
  }

  mc_control::MCGlobalController & controller;
  std::shared_ptr<ros::NodeHandle> nh;
  std::vector<ros::ServiceServer> services;
};

MCControlTCPService::MCControlTCPService(mc_control::MCGlobalController & controller) :
  impl(new MCControlTCPServiceImpl(controller))
{
}

MCControlTCPService::~MCControlTCPService()
{
}
