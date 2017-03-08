#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <mc_rtc/config.h>

#include <mc_rtc/ros_api.h>

#include <SpaceVecAlg/SpaceVecAlg>

namespace ros
{
  class NodeHandle;
}

namespace mc_rbdyn
{
  struct Robot;
}

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
   * \param gripperJ List of gripper joints managed by mc_rtc
   *
   * \param gripperQ Actual gripper values for the gripper joints managed
   * by mc_rtc
   *
   */
  static void update_robot_publisher(const std::string& publisher, double dt, const mc_rbdyn::Robot & robot, const std::map<std::string, std::vector<std::string>> & gripperJ, const std::map<std::string, std::vector<double>> & gripperQ);

  /*! \brief Reset the IMU offset in publication */
  static void reset_imu_offset();

  /*! \brief Stop ROS */
  static void shutdown();
private:
  static std::unique_ptr<ROSBridgeImpl> impl;
};

#ifdef MC_RTC_HAS_ROS

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
   */
  RobotPublisher(const std::string & prefix, unsigned int rate);

  /*! \brief Destructor */
  ~RobotPublisher();

  /*! \brief Update the publisher */
  void update(double dt, const mc_rbdyn::Robot & robot, const std::map<std::string, std::vector<std::string>> & gripperJ, const std::map<std::string, std::vector<double>> & gripperQ);

  /*! \brief Reset the IMU offset in publication */
  void reset_imu_offset();
private:
  std::unique_ptr<RobotPublisherImpl> impl;
};

#endif

}
