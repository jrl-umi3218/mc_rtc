#include <mc_rtc/ros.h>
#include <mc_rtc/config.h>
#include <mc_rbdyn/Robots.h>

#include <mc_rtc/logging.h>

#include <RBDyn/FK.h>

#ifdef MC_RTC_HAS_ROS
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf2_ros/transform_broadcaster.h>

#include <mutex>
#include <queue>
#include <thread>
#endif

#ifdef MC_RTC_HAS_ROS
namespace mc_rtc
{

inline geometry_msgs::TransformStamped PT2TF(const sva::PTransformd & X, const ros::Time & tm, const std::string & from, const std::string & to, unsigned int seq)
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

struct RobotPublisherImpl
{
public:
  RobotPublisherImpl(ros::NodeHandle & nh, const std::string& prefix, unsigned int rate)
  : nh(nh),
    j_state_pub(this->nh.advertise<sensor_msgs::JointState>(prefix+"joint_states", 1)),
    imu_pub(this->nh.advertise<sensor_msgs::Imu>(prefix+"imu", 1)),
    odom_pub(this->nh.advertise<nav_msgs::Odometry>(prefix+"odom", 1)),
    tf_caster(),
    prefix(prefix),
    running(true), seq(0), msgs(),
    rate(rate),
    th(std::bind(&RobotPublisherImpl::publishThread, this))
  {
  }

  ~RobotPublisherImpl()
  {
    running = false;
    th.join();
  }

  void update(double dt, const mc_rbdyn::Robot & robot, const std::map<std::string, std::vector<std::string>> & gJs, const std::map<std::string, std::vector<double>> & gQs)
  {
    unsigned int skip = static_cast<unsigned int>(ceil(1/(rate*dt)));
    if(++seq % skip) { return; }
    ros::Time tm = ros::Time::now();
    sensor_msgs::JointState msg;
    sensor_msgs::Imu imu;
    std::vector<geometry_msgs::TransformStamped> tfs;

    rbd::MultiBodyConfig mbc = robot.mbc();
    for(const auto & g : gJs)
    {
      const auto & gName = g.first;
      const auto & gJoints = g.second;
      const auto & gQ = gQs.at(gName);
      for(size_t i = 0; i < gJoints.size(); ++i)
      {
        const auto & j = gJoints[i];
        const auto & q = gQ[i];
        if(robot.hasJoint(j) && mbc.q[robot.jointIndexByName(j)].size() > 0)
        {
          mbc.q[robot.jointIndexByName(j)][0] = q;
        }
      }
    }
    rbd::forwardKinematics(robot.mb(), mbc);

    msg.header.seq = seq;
    msg.header.stamp = tm;
    msg.header.frame_id = "";
    msg.name.reserve(robot.mb().nrJoints() - 1);
    msg.position.reserve(robot.mb().nrJoints() - 1);
    msg.velocity.reserve(robot.mb().nrJoints() - 1);
    msg.effort.reserve(robot.mb().nrJoints() - 1);
    for(const auto & j : robot.mb().joints())
    {
      if(j.dof() == 1)
      {
        auto jIdx = robot.jointIndexByName(j.name());
        if(mbc.q[jIdx].size() > 0)
        {
          msg.name.push_back(j.name());
          msg.position.push_back(mbc.q[jIdx][0]);
          msg.velocity.push_back(mbc.alpha[jIdx][0]);
          msg.effort.push_back(mbc.jointTorque[jIdx][0]);
        }
      }
    }

    imu.header = msg.header;
    const auto & gsensor = robot.bodySensor().acceleration();
    imu.linear_acceleration.x = gsensor.x();
    imu.linear_acceleration.y = gsensor.y();
    imu.linear_acceleration.z = gsensor.z();

    nav_msgs::Odometry odom;
    odom.header = msg.header;
    odom.header.frame_id = robot.bodySensor().parentBody();
    odom.child_frame_id = "robot_odom";
    const auto & odom_p = robot.bodySensor().position();
    Eigen::Quaterniond odom_q = robot.bodySensor().orientation();
    odom.pose.pose.position.x = odom_p.x();
    odom.pose.pose.position.y = odom_p.y();
    odom.pose.pose.position.z = odom_p.z();
    odom.pose.pose.orientation.w = odom_q.w();
    odom.pose.pose.orientation.x = odom_q.x();
    odom.pose.pose.orientation.y = odom_q.y();
    odom.pose.pose.orientation.z = odom_q.z();
    odom.pose.covariance.fill(0);
    /* Provide linear and angular velocity */
    const auto & vel = robot.bodySensor().linearVelocity();
    odom.twist.twist.linear.x = vel.x();
    odom.twist.twist.linear.y = vel.y();
    odom.twist.twist.linear.z = vel.z();
    const auto & rate = robot.bodySensor().angularVelocity();
    odom.twist.twist.angular.x = rate.x();
    odom.twist.twist.angular.y = rate.y();
    odom.twist.twist.angular.z = rate.z();
    odom.twist.covariance.fill(0);

    std::vector<geometry_msgs::WrenchStamped> ros_wrenches;
    for(const auto & fs : robot.forceSensors())
    {
      const std::string & name = fs.name();
      const sva::ForceVecd & wrench_sva = fs.wrench();
      geometry_msgs::WrenchStamped wrench_msg;
      wrench_msg.header = msg.header;
      wrench_msg.header.frame_id = name;
      wrench_msg.wrench.force.x = wrench_sva.force().x();
      wrench_msg.wrench.force.y = wrench_sva.force().y();
      wrench_msg.wrench.force.z = wrench_sva.force().z();
      wrench_msg.wrench.torque.x = wrench_sva.couple().x();
      wrench_msg.wrench.torque.y = wrench_sva.couple().y();
      wrench_msg.wrench.torque.z = wrench_sva.couple().z();
      ros_wrenches.push_back(wrench_msg);
      if(!robot.hasBody(fs.name()))
      {
        tfs.push_back(PT2TF(fs.X_p_f(), tm, prefix + fs.parentBody(), prefix + name, seq));
      }
    }

    tfs.push_back(PT2TF(robot.bodyTransform(robot.mb().body(0).name())*mbc.parentToSon[0], tm, std::string("robot_map"), prefix+robot.mb().body(0).name(), seq));
    for(int j = 1; j < robot.mb().nrJoints(); ++j)
    {
      const auto & predIndex = robot.mb().predecessor(j);
      const auto & succIndex = robot.mb().successor(j);
      const auto & predName = robot.mb().body(predIndex).name();
      const auto & succName = robot.mb().body(succIndex).name();
      const auto & X_predp_pred = robot.bodyTransform(predName);
      const auto & X_succp_succ = robot.bodyTransform(succName);
      tfs.push_back(PT2TF(X_succp_succ*mbc.parentToSon[static_cast<unsigned int>(j)]*X_predp_pred.inv(), tm, prefix + predName, prefix + succName, seq));
    }

    if(seq % skip == 0)
    {
      msgs.push({msg, tfs, imu, odom, ros_wrenches});
    }
  }

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
    sensor_msgs::Imu imu;
    nav_msgs::Odometry odom;
    std::vector<geometry_msgs::WrenchStamped> wrenches;
  };

  bool running;
  uint32_t seq;
  std::queue<RobotStateData> msgs;
  unsigned int rate;
  std::thread th;

  void publishThread()
  {
    ros::Rate rt(rate);
    while(running && ros::ok())
    {
      while(msgs.size())
      {
        const auto & msg = msgs.front();
        try
        {
          j_state_pub.publish(msg.js);
          imu_pub.publish(msg.imu);
          odom_pub.publish(msg.odom);
          tf_caster.sendTransform(msg.tfs);
          for (const auto & wrench : msg.wrenches)
          {
            const std::string & sensor_name = wrench.header.frame_id;
            if (wrenches_pub.count(sensor_name) == 0)
            {
              wrenches_pub.insert({
                          sensor_name,
                          this->nh.advertise<geometry_msgs::WrenchStamped>(prefix + "force/" + sensor_name, 1)});
            }
            wrenches_pub[sensor_name].publish(wrench);
          }
        }
        catch(const ros::serialization::StreamOverrunException & e)
        {
          LOG_ERROR("EXCEPTION WHILE PUBLISHING STATE")
          LOG_WARNING(e.what())
        }
        msgs.pop();
      }
      rt.sleep();
    }
  }
};

RobotPublisher::RobotPublisher(const std::string & prefix, unsigned int rate)
  : impl(nullptr)
{
  auto nh = ROSBridge::get_node_handle();
  if(nh)
  {
    impl.reset(new RobotPublisherImpl(*nh, prefix, rate));
  }
}

RobotPublisher::~RobotPublisher()
{
}

void RobotPublisher::update(double dt, const mc_rbdyn::Robot & robot, const std::map<std::string, std::vector<std::string>> & gripperJ, const std::map<std::string, std::vector<double>> & gripperQ)
{
  if(impl)
  {
    impl->update(dt, robot, gripperJ, gripperQ);
  }
}

inline bool ros_init(const std::string & name)
{
  int argc = 0;
  char * argv[] = {0};
  ros::init(argc, argv, name.c_str());
  if(!ros::master::check())
  {
    LOG_WARNING("ROS master is not available, continue without ROS functionalities")
    return false;
  }
  return true;
}

struct ROSBridgeImpl
{
  ROSBridgeImpl()
  : ros_is_init(ros_init("mc_rtc")),
    nh(ros_is_init ? new ros::NodeHandle() : 0)
  {
  }
  bool ros_is_init;
  std::shared_ptr<ros::NodeHandle> nh;
  std::map<std::string, std::shared_ptr<RobotPublisher>> rpubs;
  unsigned int publish_rate = 100;
};

std::unique_ptr<ROSBridgeImpl> ROSBridge::impl = std::unique_ptr<ROSBridgeImpl>(new ROSBridgeImpl());

std::shared_ptr<ros::NodeHandle> ROSBridge::get_node_handle()
{
  return impl->nh;
}

void ROSBridge::set_publisher_timestep(double timestep)
{
  impl->publish_rate = static_cast<unsigned int>(floor(1/timestep));
}

void ROSBridge::update_robot_publisher(const std::string& publisher, double dt, const mc_rbdyn::Robot & robot, const std::map<std::string, std::vector<std::string>> & gJ, const std::map<std::string, std::vector<double>> & gQ)
{
  if(impl->rpubs.count(publisher) == 0)
  {
    impl->rpubs[publisher] = std::make_shared<RobotPublisher>(publisher + "/", impl->publish_rate);
  }
  impl->rpubs[publisher]->update(dt, robot, gJ, gQ);
}

void ROSBridge::shutdown()
{
  ros::shutdown();
}

}
#else
namespace ros
{
  class NodeHandle {};
}

namespace mc_rtc
{

struct ROSBridgeImpl
{
  ROSBridgeImpl() : nh(0) {}
  std::shared_ptr<ros::NodeHandle> nh;
};

std::unique_ptr<ROSBridgeImpl> ROSBridge::impl = std::unique_ptr<ROSBridgeImpl>(new ROSBridgeImpl());

std::shared_ptr<ros::NodeHandle> ROSBridge::get_node_handle()
{
  return impl->nh;
}

void ROSBridge::set_publisher_timestep(double timestep)
{
}

void ROSBridge::update_robot_publisher(const std::string&, double, const mc_rbdyn::Robot &, const std::map<std::string, std::vector<std::string>> &, const std::map<std::string, std::vector<double>> &)
{
}

void ROSBridge::shutdown()
{
}

}
#endif
