/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/Robots.h>
#include <mc_rtc/config.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/utils.h>
#include <mc_rtc_ros/ros.h>

#include <RBDyn/FK.h>

#ifdef MC_RTC_ROS_IS_ROS2
#  include <geometry_msgs/msg/wrench_stamped.hpp>
#  include <mc_rtc_msgs/msg/joint_sensors.hpp>
#  include <nav_msgs/msg/odometry.hpp>
#  include <sensor_msgs/msg/imu.hpp>
#  include <sensor_msgs/msg/joint_state.hpp>
#  include <std_msgs/msg/string.hpp>

#  include <rclcpp/rclcpp.hpp>
#else
#  include <geometry_msgs/WrenchStamped.h>
#  include <mc_rtc_msgs/JointSensors.h>
#  include <nav_msgs/Odometry.h>
#  include <sensor_msgs/Imu.h>
#  include <sensor_msgs/JointState.h>

#  include <ros/ros.h>
#endif

#include <tf2_ros/transform_broadcaster.h>

#include <fstream>
#include <thread>

#ifdef MC_RTC_ROS_IS_ROS2

namespace ros
{

static inline bool ok()
{
  return rclcpp::ok();
}

} // namespace ros

#endif

namespace mc_rtc
{

#ifdef MC_RTC_ROS_IS_ROS2

using NodeHandle = rclcpp::Node;
template<typename MessageT>
using Publisher = std::shared_ptr<rclcpp::Publisher<MessageT>>;
using Time = rclcpp::Time;

using Imu = sensor_msgs::msg::Imu;
using JointSensors = mc_rtc_msgs::msg::JointSensors;
using JointState = sensor_msgs::msg::JointState;
using Odometry = nav_msgs::msg::Odometry;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using WrenchStamped = geometry_msgs::msg::WrenchStamped;

#else

using NodeHandle = ros::NodeHandle;
template<typename MessageT>
using Publisher = ros::Publisher;
using Time = ros::Time;

using Imu = sensor_msgs::Imu;
using JointSensors = mc_rtc_msgs::JointSensors;
using JointState = sensor_msgs::JointState;
using Odometry = nav_msgs::Odometry;
using TransformStamped = geometry_msgs::TransformStamped;
using WrenchStamped = geometry_msgs::WrenchStamped;

#endif

inline TransformStamped PT2TF(const sva::PTransformd & X,
                              const Time & tm,
                              const std::string & from,
                              const std::string & to,
                              [[maybe_unused]] unsigned int seq)
{
  TransformStamped msg;
#ifndef MC_RTC_ROS_IS_ROS2
  msg.header.seq = seq;
#endif
  msg.header.stamp = tm;
  msg.header.frame_id = from;
  msg.child_frame_id = to;

  Eigen::Vector4d q = Eigen::Quaterniond(X.rotation()).inverse().coeffs();
  q.normalize();
  const Eigen::Vector3d & t = X.translation();

  msg.transform.translation.x = t.x();
  msg.transform.translation.y = t.y();
  msg.transform.translation.z = t.z();

  msg.transform.rotation.w = q.w();
  msg.transform.rotation.x = q.x();
  msg.transform.rotation.y = q.y();
  msg.transform.rotation.z = q.z();

  return msg;
}

inline void update_tf(TransformStamped & msg, const sva::PTransformd & X)
{
  Eigen::Vector4d q = Eigen::Quaterniond(X.rotation()).inverse().coeffs();
  const Eigen::Vector3d & t = X.translation();

  msg.transform.translation.x = t.x();
  msg.transform.translation.y = t.y();
  msg.transform.translation.z = t.z();

  msg.transform.rotation.w = q.w();
  msg.transform.rotation.x = q.x();
  msg.transform.rotation.y = q.y();
  msg.transform.rotation.z = q.z();
}
inline void update_tf(TransformStamped & msg,
                      const sva::PTransformd & X,
                      const Time & tm,
                      const std::string & from,
                      const std::string & to,
                      [[maybe_unused]] unsigned int seq)
{
  update_tf(msg, X);
#ifndef MC_RTC_ROS_IS_ROS2
  msg.header.seq = seq;
#endif
  msg.header.stamp = tm;
  msg.header.frame_id = from;
  msg.child_frame_id = to;
}

struct RobotPublisherImpl
{
  RobotPublisherImpl(NodeHandle & nh, const std::string & prefix, double rate, double dt);

  ~RobotPublisherImpl();

  void init(const mc_rbdyn::Robot & robot, bool use_real);

  void update(double dt, const mc_rbdyn::Robot & robot);

  void set_rate(double rate);

private:
  NodeHandle & nh;
#if MC_RTC_ROS_IS_ROS2
  std::unique_ptr<rclcpp::Rate> rosRate;
  Publisher<std_msgs::msg::String> paramsTopic;
  Publisher<std_msgs::msg::String> descriptionTopic;
#else
  ros::Rate rosRate;
#endif
  Publisher<JointState> j_state_pub;
  Publisher<Imu> imu_pub;
  Publisher<JointSensors> j_sensor_pub;
  Publisher<Odometry> odom_pub;
  std::map<std::string, Publisher<WrenchStamped>> wrenches_pub;
  tf2_ros::TransformBroadcaster tf_caster;
  std::string prefix;

  struct RobotStateData
  {
    JointState js;
    JointSensors j_sensors;
    std::vector<TransformStamped> tfs;
    std::vector<TransformStamped> surface_tfs;
    Imu imu;
    Odometry odom;
    std::vector<WrenchStamped> wrenches;
  };

  /* Hold the address of the last init/update call */
  const mc_rbdyn::Robot * previous_robot = nullptr;
  RobotStateData data;
  bool use_real;

  /** Publication details */
  bool running;
  uint32_t seq;
  CircularBuffer<RobotStateData, 1024> msgs;
  double rate;
  unsigned int skip;
  std::thread th;

  void publishThread();

  void add_force_sensor(const mc_rbdyn::Robot & robot, const mc_rbdyn::ForceSensor & fs)
  {
#ifdef MC_RTC_ROS_IS_ROS2
    auto tm = nh.now();
#else
    auto tm = Time::now();
#endif
    const std::string & name = fs.name();
    data.wrenches.emplace_back();
    auto & msg = data.wrenches.back();
    msg.header.frame_id = prefix + name;
    if(!robot.hasBody(fs.name()))
    {
      data.tfs.push_back(PT2TF(fs.X_p_f(), tm, prefix + fs.parentBody(), prefix + name, 0));
    }
  }
};

RobotPublisherImpl::RobotPublisherImpl(NodeHandle & nh, const std::string & prefix, double rate, double dt)
: nh(nh),
#if MC_RTC_ROS_IS_ROS2
  rosRate(std::make_unique<rclcpp::Rate>(rate)),
  paramsTopic(
      this->nh.create_publisher<std_msgs::msg::String>(prefix + "robot_module", rclcpp::QoS(1).transient_local())),
  descriptionTopic(
      this->nh.create_publisher<std_msgs::msg::String>(prefix + "robot_description", rclcpp::QoS(1).transient_local())),
  j_state_pub(this->nh.create_publisher<JointState>(prefix + "joint_states", 1)),
  imu_pub(this->nh.create_publisher<Imu>(prefix + "imu", 1)),
  j_sensor_pub(this->nh.create_publisher<JointSensors>(prefix + "joint_sensors", 1)),
  odom_pub(this->nh.create_publisher<Odometry>(prefix + "odom", 1)), tf_caster(nh),
#else
  rosRate(rate), j_state_pub(this->nh.advertise<JointState>(prefix + "joint_states", 1)),
  imu_pub(this->nh.advertise<Imu>(prefix + "imu", 1)),
  j_sensor_pub(this->nh.advertise<JointSensors>(prefix + "joint_sensors", 1)),
  odom_pub(this->nh.advertise<Odometry>(prefix + "odom", 1)), tf_caster(),
#endif
  prefix(prefix), use_real(false), running(true), seq(0), msgs(), rate(rate),
  skip(static_cast<unsigned int>(ceil(1 / (rate * dt)))), th(std::bind(&RobotPublisherImpl::publishThread, this))
{
}

RobotPublisherImpl::~RobotPublisherImpl()
{
  running = false;
  th.join();
}

void RobotPublisherImpl::init(const mc_rbdyn::Robot & robot, bool use_real)
{
  if(&robot == previous_robot) { return; }
  this->use_real = use_real;
  previous_robot = &robot;

  // Reset data
  data = RobotStateData();

#ifdef MC_RTC_ROS_IS_ROS2
  auto tm = nh.now();
#else
  auto tm = Time::now();
#endif

  data.js.header.frame_id = "";
  for(const auto & j : robot.mb().joints())
  {
    if(j.dof() == 1)
    {
      data.js.name.emplace_back(j.name());
      data.js.position.emplace_back(0);
      data.js.velocity.emplace_back(0);
      data.js.effort.emplace_back(0);
    }
  }

  data.j_sensors.header.frame_id = "";
  data.j_sensors.header.stamp = tm;
  for(const auto & js : robot.jointSensors())
  {
    data.j_sensors.name.emplace_back(js.joint());
    data.j_sensors.motor_temperature.emplace_back(std::numeric_limits<double>::quiet_NaN());
    data.j_sensors.driver_temperature.emplace_back(std::numeric_limits<double>::quiet_NaN());
    data.j_sensors.motor_current.emplace_back(std::numeric_limits<double>::quiet_NaN());
  }

  data.odom.header.frame_id = "robot_map";
  data.odom.child_frame_id = prefix + robot.bodySensor().parentBody();

  auto id = sva::PTransformd::Identity();
  data.tfs.push_back(PT2TF(id, tm, "robot_map", prefix + robot.mb().body(0).name(), 0));
  for(int j = 1; j < robot.mb().nrJoints(); ++j)
  {
    const auto & predIndex = robot.mb().predecessor(j);
    const auto & succIndex = robot.mb().successor(j);
    const auto & predName = robot.mb().body(predIndex).name();
    const auto & succName = robot.mb().body(succIndex).name();
    data.tfs.push_back(PT2TF(id, tm, prefix + predName, prefix + succName, 0));
  }

  for(const auto & fs : robot.forceSensors()) { add_force_sensor(robot, fs); }

  for(const auto & s : robot.surfaces())
  {
    const auto & surf = s.second;
    data.surface_tfs.push_back(
        PT2TF(surf->X_b_s(), tm, prefix + surf->bodyName(), prefix + "surfaces/" + surf->name(), 0));
  }

#ifdef MC_RTC_ROS_IS_ROS2
  {
    std_msgs::msg::String msg;
    const auto & params = robot.module().parameters();
    for(size_t i = 0; i < params.size(); ++i)
    {
      msg.data += params[i];
      if(i + 1 < params.size()) { msg.data += "#"; }
    }
    paramsTopic->publish(msg);
  }
#else
  nh.setParam(prefix + "/robot_module", robot.module().parameters());
#endif
  const auto & urdf_path = use_real ? robot.module().real_urdf() : robot.module().urdf_path;
  std::ifstream ifs(urdf_path);
  if(!ifs.is_open())
  {
    mc_rtc::log::error("{} URDF: {} is not readable", robot.name(), urdf_path);
    return;
  }
  std::stringstream urdf;
  urdf << ifs.rdbuf();
#ifdef MC_RTC_ROS_IS_ROS2
  {
    std_msgs::msg::String msg;
    msg.data = urdf.str();
    descriptionTopic->publish(msg);
  }
#else
  nh.setParam(prefix + "/robot_description", urdf.str());
#endif
}

void RobotPublisherImpl::update(double, const mc_rbdyn::Robot & robot)
{
  if(&robot != previous_robot) { init(robot, use_real); }

  if(++seq % skip) { return; }

#ifdef MC_RTC_ROS_IS_ROS2
  auto tm = nh.now();
#else
  auto tm = Time::now();
#endif

  const auto & mb = robot.mb();
  size_t tfs_i = 0;

  const auto & mbc = robot.mbc();
#ifndef MC_RTC_ROS_IS_ROS2
  data.js.header.seq = seq;
#endif
  data.js.header.stamp = tm;
  {
    size_t jI = 0;
    size_t jIdx = 0;
    for(const auto & j : mb.joints())
    {
      if(j.dof() == 1)
      {
        data.js.position[jI] = mbc.q[jIdx][0];
        data.js.velocity[jI] = mbc.alpha[jIdx][0];
        data.js.effort[jI] = mbc.jointTorque[jIdx][0];
        jI++;
      }
      jIdx += 1;
    }
  }

#ifndef MC_RTC_ROS_IS_ROS2
  data.j_sensors.header.seq = seq;
  data.js.header.seq = seq;
#endif
  data.j_sensors.header.stamp = tm;
  {
    size_t jsI = 0;
    for(const auto & js : robot.jointSensors())
    {
      data.j_sensors.motor_temperature[jsI] = js.motorTemperature();
      data.j_sensors.driver_temperature[jsI] = js.driverTemperature();
      data.j_sensors.motor_current[jsI] = js.motorCurrent();
      jsI++;
    }
  }

  data.imu.header = data.js.header;
  const auto & imu_linear_acceleration = robot.bodySensor().linearAcceleration();
  data.imu.linear_acceleration.x = imu_linear_acceleration.x();
  data.imu.linear_acceleration.y = imu_linear_acceleration.y();
  data.imu.linear_acceleration.z = imu_linear_acceleration.z();
  const auto & imu_angular_velocity = robot.bodySensor().angularVelocity();
  data.imu.angular_velocity.x = imu_angular_velocity.x();
  data.imu.angular_velocity.y = imu_angular_velocity.y();
  data.imu.angular_velocity.z = imu_angular_velocity.z();
  const auto & imu_orientation = robot.bodySensor().orientation();
  data.imu.orientation.w = imu_orientation.w();
  data.imu.orientation.x = imu_orientation.x();
  data.imu.orientation.y = imu_orientation.y();
  data.imu.orientation.z = imu_orientation.z();

#ifndef MC_RTC_ROS_IS_ROS2
  data.odom.header.seq = data.js.header.seq;
#endif
  data.odom.header.stamp = data.js.header.stamp;
  const auto & odom_p = robot.bodySensor().position();
  Eigen::Quaterniond odom_q = robot.bodySensor().orientation();
  data.odom.pose.pose.position.x = odom_p.x();
  data.odom.pose.pose.position.y = odom_p.y();
  data.odom.pose.pose.position.z = odom_p.z();
  data.odom.pose.pose.orientation.w = odom_q.w();
  data.odom.pose.pose.orientation.x = odom_q.x();
  data.odom.pose.pose.orientation.y = odom_q.y();
  data.odom.pose.pose.orientation.z = odom_q.z();
  data.odom.pose.covariance.fill(0);
  /* Provide linear and angular velocity */
  const auto & vel = robot.bodySensor().linearVelocity();
  data.odom.twist.twist.linear.x = vel.x();
  data.odom.twist.twist.linear.y = vel.y();
  data.odom.twist.twist.linear.z = vel.z();
  const auto & rate = robot.bodySensor().angularVelocity();
  data.odom.twist.twist.angular.x = rate.x();
  data.odom.twist.twist.angular.y = rate.y();
  data.odom.twist.twist.angular.z = rate.z();
  data.odom.twist.covariance.fill(0);

  {
    size_t wrench_i = 0;
    for(const auto & fs : robot.forceSensors())
    {
      if(wrench_i >= data.wrenches.size()) { add_force_sensor(robot, fs); }
      auto & msg = data.wrenches[wrench_i];
      const sva::ForceVecd & wrench_sva = fs.wrench();
      msg.header.stamp = data.js.header.stamp;
#ifndef MC_RTC_ROS_IS_ROS2
      msg.header.seq = data.js.header.seq;
#endif
      msg.wrench.force.x = wrench_sva.force().x();
      msg.wrench.force.y = wrench_sva.force().y();
      msg.wrench.force.z = wrench_sva.force().z();
      msg.wrench.torque.x = wrench_sva.couple().x();
      msg.wrench.torque.y = wrench_sva.couple().y();
      msg.wrench.torque.z = wrench_sva.couple().z();
      wrench_i++;
    }
  }

  update_tf(data.tfs[tfs_i++], robot.bodyTransform(0) * mbc.parentToSon[0]);
  for(int j = 1; j < mb.nrJoints(); ++j)
  {
    const auto & predIndex = mb.predecessor(j);
    const auto & succIndex = mb.successor(j);
    const auto & X_predp_pred = robot.bodyTransform(predIndex);
    const auto & X_succp_succ = robot.bodyTransform(succIndex);
    update_tf(data.tfs[tfs_i++], X_succp_succ * mbc.parentToSon[static_cast<size_t>(j)] * X_predp_pred.inv());
  }

  {
    data.surface_tfs.resize(robot.surfaces().size());
    size_t surf_i = 0;
    for(const auto & s : robot.surfaces())
    {
      const auto & surf = s.second;
      data.surface_tfs[surf_i] =
          PT2TF(surf->X_b_s(), tm, prefix + surf->bodyName(), prefix + "surfaces/" + surf->name(), 0);
      data.surface_tfs[surf_i].header.stamp = data.js.header.stamp;
#ifndef MC_RTC_ROS_IS_ROS2
      data.surface_tfs[surf_i].header.seq = data.js.header.seq;
#endif
      ++surf_i;
    }
  }

  for(auto & tf : data.tfs)
  {
    tf.header.stamp = data.js.header.stamp;
#ifndef MC_RTC_ROS_IS_ROS2
    tf.header.seq = data.js.header.seq;
#endif
  }

  if(!msgs.push(data)) { mc_rtc::log::error("Full ROS message publishing queue"); }
}

RobotPublisher::RobotPublisher(const std::string & prefix, double rate, double dt) : impl(nullptr)
{
  auto nh = ROSBridge::get_node_handle();
  if(nh) { impl.reset(new RobotPublisherImpl(*nh, prefix, rate, dt)); }
}

RobotPublisher::~RobotPublisher() {}

void RobotPublisher::init(const mc_rbdyn::Robot & robot, bool use_real)
{
  if(impl) { impl->init(robot, use_real); }
}

void RobotPublisher::update(double dt, const mc_rbdyn::Robot & robot)
{
  if(impl) { impl->update(dt, robot); }
}

void RobotPublisher::set_rate(double rate)
{
  if(impl) { impl->set_rate(rate); }
}

void RobotPublisherImpl::publishThread()
{
  RobotStateData msg;
  while(running && ros::ok())
  {
    while(msgs.pop(msg))
    {
      try
      {
#ifdef MC_RTC_ROS_IS_ROS2
        j_state_pub->publish(msg.js);
        j_sensor_pub->publish(msg.j_sensors);
        imu_pub->publish(msg.imu);
        odom_pub->publish(msg.odom);
#else
        j_state_pub.publish(msg.js);
        j_sensor_pub.publish(msg.j_sensors);
        imu_pub.publish(msg.imu);
        odom_pub.publish(msg.odom);
#endif
        tf_caster.sendTransform(msg.tfs);
        tf_caster.sendTransform(msg.surface_tfs);
        for(const auto & wrench : msg.wrenches)
        {
          const std::string & sensor_name = wrench.header.frame_id.substr(prefix.length());
          if(wrenches_pub.count(sensor_name) == 0)
          {
#if MC_RTC_ROS_IS_ROS2
            wrenches_pub.insert(
                {sensor_name, this->nh.create_publisher<WrenchStamped>(prefix + "force/" + sensor_name, 1)});
#else
            wrenches_pub.insert({sensor_name, this->nh.advertise<WrenchStamped>(prefix + "force/" + sensor_name, 1)});
#endif
          }
#if MC_RTC_ROS_IS_ROS2
          wrenches_pub[sensor_name]->publish(wrench);
#else
          wrenches_pub[sensor_name].publish(wrench);
#endif
        }
      }
#ifdef MC_RTC_ROS_IS_ROS2
      catch(const std::exception & e)
#else
      catch(const ros::serialization::StreamOverrunException & e)
#endif
      {
        mc_rtc::log::error("EXCEPTION WHILE PUBLISHING STATE");
        mc_rtc::log::warning(e.what());
      }
    }
#ifdef MC_RTC_ROS_IS_ROS2
    rosRate->sleep();
#else
    rosRate.sleep();
#endif
  }
}

void RobotPublisherImpl::set_rate(double rateIn)
{
  double ctl_dt = 1 / (rate * skip);
  rate = rateIn;
  skip = static_cast<unsigned int>(ceil(1 / (rate * ctl_dt)));
#if MC_RTC_ROS_IS_ROS2
  rosRate = std::make_unique<rclcpp::Rate>(rate);
#else
  rosRate = ros::Rate(rate);
#endif
}

inline bool ros_init([[maybe_unused]] const std::string & name)
{
  if(ros::ok()) { return true; }
  int argc = 0;
#ifdef MC_RTC_ROS_IS_ROS2
  rclcpp::init(argc, nullptr);
#else
  ros::init(argc, nullptr, name.c_str(), ros::init_options::NoSigintHandler);
  if(!ros::master::check())
  {
    mc_rtc::log::warning("ROS master is not available, continue without ROS functionalities");
    return false;
  }
#endif
  return true;
}

struct ROSBridgeImpl
{
  ROSBridgeImpl()
  : ros_is_init(ros_init("mc_rtc")),
#ifdef MC_RTC_ROS_IS_ROS2
    nh(ros_is_init ? rclcpp::Node::make_shared("mc_rtc") : 0)
#else
    nh(ros_is_init ? new ros::NodeHandle() : 0)
#endif
  {
  }
  bool ros_is_init;
  std::shared_ptr<NodeHandle> nh;
  std::map<std::string, std::shared_ptr<RobotPublisher>> rpubs;
  double publish_rate = 100;
};

ROSBridgeImpl & ROSBridge::impl_()
{
  static std::unique_ptr<ROSBridgeImpl> impl{new ROSBridgeImpl()};
  return *impl;
}

NodeHandlePtr ROSBridge::get_node_handle()
{
  static auto & impl = impl_();
  return impl.nh;
}

void ROSBridge::set_publisher_timestep(double timestep)
{
  static auto & impl = impl_();
  impl.publish_rate = 1 / timestep;
  for(auto & rpub_it : impl.rpubs)
  {
    auto & rpub = *rpub_it.second.get();
    rpub.set_rate(impl.publish_rate);
  }
}

void ROSBridge::init_robot_publisher(const std::string & publisher,
                                     double dt,
                                     const mc_rbdyn::Robot & robot,
                                     bool use_real)
{
  static auto & impl = impl_();
  if(impl.rpubs.count(publisher) == 0)
  {
    impl.rpubs[publisher] = std::make_shared<RobotPublisher>(publisher + "/", impl.publish_rate, dt);
  }
  impl.rpubs[publisher]->init(robot, use_real);
}

void ROSBridge::update_robot_publisher(const std::string & publisher, double dt, const mc_rbdyn::Robot & robot)
{
  static auto & impl = impl_();
  if(impl.rpubs.count(publisher) == 0)
  {
    impl.rpubs[publisher] = std::make_shared<RobotPublisher>(publisher + "/", impl.publish_rate, dt);
    impl.rpubs[publisher]->init(robot);
  }
  impl.rpubs[publisher]->update(dt, robot);
}

void ROSBridge::stop_robot_publisher(const std::string & publisher)
{
  static auto & impl = impl_();
  auto it = impl.rpubs.find(publisher);
  if(it == impl.rpubs.end()) { return; }
  impl.rpubs.erase(it);
}

void ROSBridge::shutdown()
{
#ifdef MC_RTC_ROS_IS_ROS2
  rclcpp::shutdown();
#else
  ros::shutdown();
#endif
}

} // namespace mc_rtc
