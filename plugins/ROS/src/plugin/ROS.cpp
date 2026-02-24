#include "ROS.h"

#include <mc_rtc_ros/ros.h>

#include <mc_control/GlobalPluginMacros.h>

#include <mc_tasks_ros/LookAtTFTask.h>

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
    conf("env", publish_env);
    conf("real", publish_real);
    conf("timestep", publish_timestep);
  }
  mc_rtc::ROSBridge::set_publisher_timestep(publish_timestep);
  services_.reset(new ROSServices(mc_rtc::ROSBridge::get_node_handle(), controller));
  reset(controller);
}

void ROSPlugin::reset(mc_control::MCGlobalController & controller)
{
  auto publish_robots = [&controller](const std::string & prefix, mc_rbdyn::Robots & robots, bool use_real)
  {
    for(const auto & r : robots)
    {
      mc_rtc::ROSBridge::init_robot_publisher(prefix + r.name(), controller.timestep(), r, use_real);
    }
  };

  publish_robots("control/", controller.controller().robots(), false);

  if(publish_real) { publish_robots("real/", controller.controller().realRobots(), true); }
}

void ROSPlugin::after(mc_control::MCGlobalController & controller)
{
  auto update_robots = [this, &controller](const std::string & prefix, mc_rbdyn::Robots & robots)
  {
    for(auto & r : robots) { mc_rtc::ROSBridge::update_robot_publisher(prefix + r.name(), controller.timestep(), r); }
    published_topics = robots.size() - 1;
  };
  update_robots("control/", controller.controller().robots());
  if(publish_real) { update_robots("real/", controller.controller().realRobots()); }

  mc_rtc::ROSBridge::remove_extra_robot_publishers(controller.controller().robots());
}

ROSPlugin::~ROSPlugin()
{
  mc_rtc::ROSBridge::stop_robot_publisher("control");
  mc_rtc::ROSBridge::stop_robot_publisher("real");
  for(size_t i = 0; i < published_topics; ++i)
  {
    mc_rtc::ROSBridge::stop_robot_publisher("control/env_" + std::to_string(i + 1));
    mc_rtc::ROSBridge::stop_robot_publisher("real/env_" + std::to_string(i + 1));
  }
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("ROS", mc_plugin::ROSPlugin)
