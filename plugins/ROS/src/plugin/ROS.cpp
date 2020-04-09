#include "ROS.h"

#include <mc_rtc/ros.h>

#include <mc_control/GlobalPluginMacros.h>

#include <mc_tasks/LookAtTFTask.h>

namespace mc_plugin
{

// This is useless but ensure we bring in LookAtTFTask into the library
void ROSPlugin::build(mc_control::MCGlobalController & controller)
{
  mc_tasks::LookAtTFTask task("body", Eigen::Vector3d::UnitZ(), "source", "target", controller.controller().robots(),
                              0);
}

void ROSPlugin::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  if(config.has("publish"))
  {
    auto conf = config("publish");
    conf("control", publish_control);
    conf("env", publish_control);
    conf("real", publish_real);
    conf("timestep", publish_timestep);
  }
  mc_rtc::ROSBridge::set_publisher_timestep(publish_timestep);
  services_.reset(new ROSServices(mc_rtc::ROSBridge::get_node_handle(), controller));
  reset(controller);
}

void ROSPlugin::reset(mc_control::MCGlobalController & controller)
{
  if(publish_control)
  {
    mc_rtc::ROSBridge::init_robot_publisher("control", controller.timestep(), controller.robot());
  }
  if(publish_env)
  {
    const auto & robots = controller.controller().robots();
    for(size_t i = 1; i < robots.robots().size(); ++i)
    {
      mc_rtc::ROSBridge::init_robot_publisher("control/env_" + std::to_string(i), controller.timestep(),
                                              robots.robot(i));
    }
  }
  if(publish_real)
  {
    const auto & real_robot = controller.controller().realRobot();
    mc_rtc::ROSBridge::init_robot_publisher("real", controller.timestep(), real_robot, true);
  }
}

void ROSPlugin::after(mc_control::MCGlobalController & controller)
{
  if(publish_control)
  {
    mc_rtc::ROSBridge::update_robot_publisher("control", controller.timestep(), controller.robot());
  }
  // Publish environment state
  if(publish_env)
  {
    const auto & robots = controller.controller().robots();
    for(size_t i = 1; i < robots.robots().size(); ++i)
    {
      mc_rtc::ROSBridge::update_robot_publisher("control/env_" + std::to_string(i), controller.timestep(),
                                                robots.robot(i));
    }
  }
  // Publish real robot
  if(publish_real)
  {
    auto & real_robot = controller.controller().realRobot();
    mc_rtc::ROSBridge::update_robot_publisher("real", controller.timestep(), real_robot);
  }
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("ROS", mc_plugin::ROSPlugin)
