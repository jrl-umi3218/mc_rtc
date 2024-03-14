/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks_ros/LookAtTFTask.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_rtc/deprecated.h>

#ifdef MC_RTC_ROS_IS_ROS2
#  include <mc_rtc_ros/ros.h>
#  include <rclcpp/rclcpp.hpp>
#endif

namespace mc_tasks
{
LookAtTFTask::LookAtTFTask(const std::string & bodyName,
                           const Eigen::Vector3d & bodyVector,
                           const std::string & sourceFrame,
                           const std::string & targetFrame,
                           const mc_rbdyn::Robots & robots,
                           unsigned int robotIndex,
                           double stiffness,
                           double weight)
: LookAtTFTask(robots.robot(robotIndex).frame(bodyName), bodyVector, sourceFrame, targetFrame, stiffness, weight)
{
}

LookAtTFTask::LookAtTFTask(const mc_rbdyn::RobotFrame & frame,
                           const Eigen::Vector3d & frameVector,
                           const std::string & sourceFrame,
                           const std::string & targetFrame,
                           double stiffness,
                           double weight)
: LookAtTask(frame, frameVector, stiffness, weight),
#ifdef MC_RTC_ROS_IS_ROS2
  tfBuffer(mc_rtc::ROSBridge::get_node_handle()->get_clock()),
#endif
  tfListener(tfBuffer), sourceFrame(sourceFrame), targetFrame(targetFrame)
{
  type_ = "lookAtTF";
  name_ = "look_at_TF_" + frame.robot().name() + "_" + frame.name() + "_" + targetFrame;
}

void LookAtTFTask::update(mc_solver::QPSolver &)
{
#ifdef MC_RTC_ROS_IS_ROS2
  geometry_msgs::msg::TransformStamped transformStamped;
#else
  geometry_msgs::TransformStamped transformStamped;
#endif
  try
  {
    // lookupTransform(target_frame, source_frame) returns the transformation
    // from target frame to source frame expressed in the
    // target frame coordinates. We want the same transformation from source
    // frame to target frame expressed in the source frame coordinates, which is
    // the inverse calling order for lookupTransform.
    transformStamped = tfBuffer.lookupTransform(sourceFrame, targetFrame,
#ifdef MC_RTC_ROS_IS_ROS2
                                                rclcpp::Time(0)
#else
                                                ros::Time(0)
#endif
    );
  }
  catch(tf2::TransformException & ex)
  {
    mc_rtc::log::error("TF2 exception in {}:\n{}", name(), ex.what());
    return;
  }
  Eigen::Vector3d target;
  target << transformStamped.transform.translation.x, transformStamped.transform.translation.y,
      transformStamped.transform.translation.z;

  LookAtTask::target(target);
}

} /* namespace mc_tasks */

namespace
{
static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "lookAtTF",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
    {
      const auto & robots = robotFromConfig(config, solver.robots(), "lookAtTF");
      Eigen::Vector3d frameVector = Eigen::Vector3d::Zero();
      const auto & frame = [&]() -> const mc_rbdyn::RobotFrame &
      {
        if(config.has("body"))
        {
          mc_rtc::log::deprecated("LookAtTFTaskLoader", "body", "frame");
          frameVector = config("bodyVector");
          return robots.frame(config("body"));
        }
        frameVector = config("frameVector");
        return robots.frame(config("frame"));
      }();
      auto t =
          std::make_shared<mc_tasks::LookAtTFTask>(frame, frameVector, config("sourceFrame"), config("targetFrame"));
      t->load(solver, config);
      return t;
    });
} // namespace
