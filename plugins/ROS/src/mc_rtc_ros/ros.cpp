/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/Robots.h>
#include <mc_rtc/config.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/ros.h>
#include <mc_rtc/utils.h>

#include <RBDyn/FK.h>

#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_broadcaster.h>

#include <fstream>
#include <thread>

namespace mc_rtc
{

inline geometry_msgs::TransformStamped PT2TF(const sva::PTransformd & X,
                                             const ros::Time & tm,
                                             const std::string & from,
                                             const std::string & to,
                                             unsigned int seq)
{
  geometry_msgs::TransformStamped msg;
  msg.header.seq = seq;
  msg.header.stamp = tm;
  msg.header.frame_id = from;
  msg.child_frame_id = to;

  Eigen::Vector4d q = Eigen::Quaterniond(X.rotation()).inverse().coeffs();
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

inline void update_tf(geometry_msgs::TransformStamped & msg, const sva::PTransformd & X)
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
inline void update_tf(geometry_msgs::TransformStamped & msg,
                      const sva::PTransformd & X,
                      const ros::Time & tm,
                      const std::string & from,
                      const std::string & to,
                      unsigned int seq)
{
  update_tf(msg, X);
  msg.header.seq = seq;
  msg.header.stamp = tm;
  msg.header.frame_id = from;
  msg.child_frame_id = to;
}

struct RobotPublisherImpl
{
  RobotPublisherImpl(ros::NodeHandle & nh, const std::string & prefix, double rate, double dt);

  ~RobotPublisherImpl();

  void init(const mc_rbdyn::Robot & robot, bool use_real);

  void update(double dt, const mc_rbdyn::Robot & robot);

private:
  ros::NodeHandle & nh;
  ros::Publisher j_state_pub;
  ros::Publisher imu_pub;
  ros::Publisher odom_pub;
  std::map<std::string, ros::Publisher> wrenches_pub;
  tf2_ros::TransformBroadcaster tf_caster;
  std::string prefix;

  struct RobotStateData
  {
    sensor_msgs::JointState js;
    std::vector<geometry_msgs::TransformStamped> tfs;
    std::vector<geometry_msgs::TransformStamped> surface_tfs;
    sensor_msgs::Imu imu;
    nav_msgs::Odometry odom;
    std::vector<geometry_msgs::WrenchStamped> wrenches;
  };

  /* Hold the address of the last init/update call */
  const mc_rbdyn::Robot * previous_robot;
  RobotStateData data;
  bool use_real;

  /** Publication details */
  bool running;
  uint32_t seq;
  CircularBuffer<RobotStateData, 128> msgs;
  double rate;
  unsigned int skip;
  std::thread th;

  void publishThread();
};

RobotPublisherImpl::RobotPublisherImpl(ros::NodeHandle & nh, const std::string & prefix, double rate, double dt)
: nh(nh), j_state_pub(this->nh.advertise<sensor_msgs::JointState>(prefix + "joint_states", 1)),
  imu_pub(this->nh.advertise<sensor_msgs::Imu>(prefix + "imu", 1)),
  odom_pub(this->nh.advertise<nav_msgs::Odometry>(prefix + "odom", 1)), tf_caster(), prefix(prefix), use_real(false),
  running(true), seq(0), msgs(), rate(rate), skip(static_cast<unsigned int>(ceil(1 / (rate * dt)))),
  th(std::bind(&RobotPublisherImpl::publishThread, this))
{
}

RobotPublisherImpl::~RobotPublisherImpl()
{
  running = false;
  th.join();
}

void RobotPublisherImpl::init(const mc_rbdyn::Robot & robot, bool use_real)
{
  if(&robot == previous_robot)
  {
    return;
  }
  this->use_real = use_real;
  previous_robot = &robot;

  // Reset data
  data = RobotStateData{};

  auto tm = ros::Time::now();

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

  for(const auto & fs : robot.forceSensors())
  {
    const std::string & name = fs.name();
    data.wrenches.emplace_back();
    auto & msg = data.wrenches.back();
    msg.header.frame_id = prefix + name;
    if(!robot.hasBody(fs.name()))
    {
      data.tfs.push_back(PT2TF(fs.X_p_f(), tm, prefix + fs.parentBody(), prefix + name, 0));
    }
  }

  for(const auto & s : robot.surfaces())
  {
    const auto & surf = s.second;
    data.surface_tfs.push_back(
        PT2TF(surf->X_b_s(), tm, prefix + surf->bodyName(), prefix + "surfaces/" + surf->name(), 0));
  }

  nh.setParam(prefix + "/robot_module", robot.module().parameters());
  const auto & urdf_path = use_real ? robot.module().real_urdf() : robot.module().urdf_path;
  std::ifstream ifs(urdf_path);
  if(!ifs.is_open())
  {
    mc_rtc::log::error("{} URDF: {} is not readable", robot.name(), urdf_path);
    return;
  }
  std::stringstream urdf;
  urdf << ifs.rdbuf();
  nh.setParam(prefix + "/robot_description", urdf.str());
}

void RobotPublisherImpl::update(double, const mc_rbdyn::Robot & robot)
{
  if(&robot != previous_robot)
  {
    init(robot, use_real);
  }

  if(++seq % skip)
  {
    return;
  }

  ros::Time tm = ros::Time::now();

  const auto & mb = robot.mb();
  size_t tfs_i = 0;

  const auto & mbc = robot.mbc();
  data.js.header.seq = seq;
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
  data.imu.orientation.x = imu_orientation.x();
  data.imu.orientation.y = imu_orientation.y();
  data.imu.orientation.z = imu_orientation.z();

  data.odom.header.seq = data.js.header.seq;
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
      auto & msg = data.wrenches[wrench_i++];
      const sva::ForceVecd & wrench_sva = fs.wrench();
      msg.header.stamp = data.js.header.stamp;
      msg.header.seq = data.js.header.seq;
      msg.wrench.force.x = wrench_sva.force().x();
      msg.wrench.force.y = wrench_sva.force().y();
      msg.wrench.force.z = wrench_sva.force().z();
      msg.wrench.torque.x = wrench_sva.couple().x();
      msg.wrench.torque.y = wrench_sva.couple().y();
      msg.wrench.torque.z = wrench_sva.couple().z();
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
      data.surface_tfs[surf_i].header.seq = data.js.header.seq;
      ++surf_i;
    }
  }

  for(auto & tf : data.tfs)
  {
    tf.header.stamp = data.js.header.stamp;
    tf.header.seq = data.js.header.seq;
  }

  if(!msgs.push(data))
  {
    mc_rtc::log::error("Full ROS message publishing queue");
  }
}

RobotPublisher::RobotPublisher(const std::string & prefix, double rate, double dt) : impl(nullptr)
{
  auto nh = ROSBridge::get_node_handle();
  if(nh)
  {
    impl.reset(new RobotPublisherImpl(*nh, prefix, rate, dt));
  }
}

RobotPublisher::~RobotPublisher() {}

void RobotPublisher::init(const mc_rbdyn::Robot & robot, bool use_real)
{
  if(impl)
  {
    impl->init(robot, use_real);
  }
}

void RobotPublisher::update(double dt, const mc_rbdyn::Robot & robot)
{
  if(impl)
  {
    impl->update(dt, robot);
  }
}

void RobotPublisherImpl::publishThread()
{
  ros::Rate rt(rate);
  RobotStateData msg;
  while(running && ros::ok())
  {
    while(msgs.pop(msg))
    {
      try
      {
        j_state_pub.publish(msg.js);
        imu_pub.publish(msg.imu);
        odom_pub.publish(msg.odom);
        tf_caster.sendTransform(msg.tfs);
        tf_caster.sendTransform(msg.surface_tfs);
        for(const auto & wrench : msg.wrenches)
        {
          const std::string & sensor_name = wrench.header.frame_id.substr(prefix.length());
          if(wrenches_pub.count(sensor_name) == 0)
          {
            wrenches_pub.insert(
                {sensor_name, this->nh.advertise<geometry_msgs::WrenchStamped>(prefix + "force/" + sensor_name, 1)});
          }
          wrenches_pub[sensor_name].publish(wrench);
        }
      }
      catch(const ros::serialization::StreamOverrunException & e)
      {
        mc_rtc::log::error("EXCEPTION WHILE PUBLISHING STATE");
        mc_rtc::log::warning(e.what());
      }
    }
    rt.sleep();
  }
}

inline bool ros_init(const std::string & name)
{
  if(ros::ok())
  {
    return true;
  }
  int argc = 0;
  char * argv[] = {0};
  ros::init(argc, argv, name.c_str(), ros::init_options::NoSigintHandler);
  if(!ros::master::check())
  {
    mc_rtc::log::warning("ROS master is not available, continue without ROS functionalities");
    return false;
  }
  return true;
}

struct ROSBridgeImpl
{
  ROSBridgeImpl() : ros_is_init(ros_init("mc_rtc")), nh(ros_is_init ? new ros::NodeHandle() : 0) {}
  bool ros_is_init;
  std::shared_ptr<ros::NodeHandle> nh;
  std::map<std::string, std::shared_ptr<RobotPublisher>> rpubs;
  double publish_rate = 100;
};

ROSBridgeImpl & ROSBridge::impl_()
{
  static std::unique_ptr<ROSBridgeImpl> impl{new ROSBridgeImpl()};
  return *impl;
}

std::shared_ptr<ros::NodeHandle> ROSBridge::get_node_handle()
{
  static auto & impl = impl_();
  return impl.nh;
}

void ROSBridge::set_publisher_timestep(double timestep)
{
  static auto & impl = impl_();
  impl.publish_rate = 1 / timestep;
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

void ROSBridge::shutdown()
{
  ros::shutdown();
}

} // namespace mc_rtc
