/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/config.h>
#include <mc_rtc/ros_api.h>

#include <SpaceVecAlg/SpaceVecAlg>

#include <Eigen/Geometry>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace ros
{
class NodeHandle;
}

namespace mc_rbdyn
{
struct Robot;
}

namespace mc_control
{
struct MCGlobalController;
} // namespace mc_control

namespace mc_rtc
{

struct ROSBridgeImpl;

/*! \brief Allows to access ROS functionalities within mc_rtc without explicit ROS dependencies */
struct MC_RTC_ROS_DLLAPI ROSBridge
{
  /*! \brief Get a ros::NodeHandle
   *
   * This function will return a nullptr if ROS is not available
   *
   * \return A shared_ptr to a ros::NodeHandle
   */
  static std::shared_ptr<ros::NodeHandle> get_node_handle();

  /** Set publisher timestep
   *
   * \param timestep Update timestep in ms
   *
   */
  static void set_publisher_timestep(double timestep);

  /** Update the robot publisher state
   *
   * \param publisher Name of the publisher
   *
   * \param dt Controller timestep
   *
   * \param robot Which robot to publish
   *
   * \param use_real Use the real URDF rather than the default one
   *
   */
  static void init_robot_publisher(const std::string & publisher,
                                   double dt,
                                   const mc_rbdyn::Robot & robot,
                                   bool use_real = false);

  /** Update the robot publisher state
   *
   * \param publisher Name of the publisher
   *
   * \param dt Controller timestep
   *
   * \param robot Which robot to publish
   *
   */
  static void update_robot_publisher(const std::string & publisher, double dt, const mc_rbdyn::Robot & robot);

  /** Stop the publication of a robot
   *
   * \param publisher Name of the publisher
   */
  static void stop_robot_publisher(const std::string & publisher);

  /*! \brief Stop ROS */
  static void shutdown();

private:
  static ROSBridgeImpl & impl_();
};

struct RobotPublisherImpl;

/*! \brief This structure is able to publish a Robot's state to ROS
 *
 * When writing a controller inside mc_rtc controller framework, one is not
 * expect to use this class. However, this is useful to have when building
 * tools around the framework.
 */
struct MC_RTC_ROS_DLLAPI RobotPublisher
{
public:
  /** Constructor
   *
   * If ROSBridge::get_node_handle returns a nullptr then this object does
   * nothing.
   *
   * \param prefix TF prefix
   *
   * \param rate Publishing rate
   *
   * \param dt Control rate
   */
  RobotPublisher(const std::string & prefix, double rate, double dt);

  /*! \brief Destructor */
  ~RobotPublisher();

  /*! \brief Initialize the publisher */
  void init(const mc_rbdyn::Robot & robot, bool use_real = false);

  /*! \brief Update the publisher */
  void update(double dt, const mc_rbdyn::Robot & robot);

  /*! \brief Reset the publishing rate */
  void set_rate(double rate);

private:
  std::unique_ptr<RobotPublisherImpl> impl;
};

} // namespace mc_rtc
