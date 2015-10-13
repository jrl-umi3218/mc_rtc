#include <mc_rtc/ros.h>
#include <mc_rtc/config.h>
#include <mc_rbdyn/robot.h>


#ifdef MC_RTC_HAS_ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_broadcaster.h>

#include <mutex>
#include <queue>
#include <thread>
#endif

namespace mc_rtc
{

#ifdef MC_RTC_HAS_ROS

static const std::vector<std::string> REF_JOINT_ORDER = {
  "RLEG_JOINT0", "RLEG_JOINT1", "RLEG_JOINT2", "RLEG_JOINT3", "RLEG_JOINT4", "RLEG_JOINT5",
  "LLEG_JOINT0", "LLEG_JOINT1", "LLEG_JOINT2", "LLEG_JOINT3", "LLEG_JOINT4", "LLEG_JOINT5",
  "CHEST_JOINT0", "CHEST_JOINT1", "HEAD_JOINT0", "HEAD_JOINT1",
  "RARM_JOINT0", "RARM_JOINT1", "RARM_JOINT2", "RARM_JOINT3", "RARM_JOINT4", "RARM_JOINT5", "RARM_JOINT6", "RARM_JOINT7",
  "LARM_JOINT0", "LARM_JOINT1", "LARM_JOINT2", "LARM_JOINT3", "LARM_JOINT4", "LARM_JOINT5", "LARM_JOINT6", "LARM_JOINT7",
  "RHAND_JOINT0", "RHAND_JOINT1", "RHAND_JOINT2", "RHAND_JOINT3", "RHAND_JOINT4",
  "LHAND_JOINT0", "LHAND_JOINT1", "LHAND_JOINT2", "LHAND_JOINT3", "LHAND_JOINT4"};

inline ros::NodeHandle ros_init(const std::string & name)
{
  int argc = 0;
  char * argv[] = {0};
  ros::init(argc, argv, name.c_str());
  if(!ros::master::check())
  {
    std::cerr << "ROS master is not available, continue without publishing" << std::endl;
    throw("ROS master is not available, continue without publishing");
  }
  return ros::NodeHandle();
}

inline geometry_msgs::TransformStamped PT2TF(const sva::PTransformd & X, const ros::Time & tm, const std::string & from, const std::string & to)
{
  geometry_msgs::TransformStamped msg;
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
  RobotPublisherImpl(const std::string & node_name)
  : nh(ros_init(node_name)),
    j_state_pub(nh.advertise<sensor_msgs::JointState>("joint_states", 1)),
    tf_caster(),
    seq(0), running(true), msgs(),
    th(std::bind(&RobotPublisherImpl::publishThread, this))
  {
  }

  ~RobotPublisherImpl()
  {
    stop();
  }

  void stop()
  {
    running = false;
    th.join();
  }
  void update(const mc_rbdyn::Robot & robot, const RTC::TimedPoint3D & p, const RTC::TimedOrientation3D & rpy)
  {
    ros::Time tm = ros::Time::now();
    sensor_msgs::JointState msg;
    std::vector<geometry_msgs::TransformStamped> tfs;

    msg.header.seq = ++seq;
    msg.header.stamp = tm;
    msg.header.frame_id = "";
    msg.name = REF_JOINT_ORDER;
    msg.position.reserve(REF_JOINT_ORDER.size());
    for(size_t i = 0; i < REF_JOINT_ORDER.size(); ++i)
    {
      msg.position.push_back(robot.mbc->q[robot.jointIndexByName(REF_JOINT_ORDER[i])][0]);
    }
    msg.velocity.resize(0);
    msg.effort.resize(0);

    tfs.push_back(PT2TF(robot.bodyTransforms.at(robot.mb->body(0).id())*robot.mbc->parentToSon[0], tm, std::string("/map"), robot.mb->body(0).name()));
    for(int j = 1; j < robot.mb->nrJoints(); ++j)
    {
      const auto & predIndex = robot.mb->predecessor(j);
      const auto & succIndex = robot.mb->successor(j);
      const auto & predName = robot.mb->body(predIndex).name();
      const auto & succName = robot.mb->body(succIndex).name();
      const auto & predId = robot.mb->body(predIndex).id();
      const auto & succId = robot.mb->body(succIndex).id();
      const auto & X_predp_pred = robot.bodyTransforms.at(predId);
      const auto & X_succp_succ = robot.bodyTransforms.at(succId);
      tfs.push_back(PT2TF(X_succp_succ*robot.mbc->parentToSon[j]*X_predp_pred.inv(), tm, predName, succName));
    }

    sva::PTransformd X_0_hl1 = robot.mbc->bodyPosW[robot.bodyIndexByName("HEAD_LINK1")];
    sva::PTransformd X_hl1_xtion = sva::PTransformd(Eigen::Quaterniond(0.995397, 1.7518e-05, 0.0950535, -0.0122609).inverse(), Eigen::Vector3d(0.09699157105, 0.0185, 0.12699543329));
    sva::PTransformd X_0_base_odom = sva::PTransformd(
                        Eigen::Quaterniond(sva::RotZ(rpy.data.r)*sva::RotY(rpy.data.p)*sva::RotX(rpy.data.y)),
                        Eigen::Vector3d(p.data.x, p.data.y, p.data.z));
    sva::PTransformd X_0_xtion = X_hl1_xtion * X_0_hl1;
    sva::PTransformd X_0_base = robot.mbc->bodyPosW[0];
    sva::PTransformd X_base_xtion = X_0_xtion * (X_0_base.inv());

    tfs.push_back(PT2TF(X_hl1_xtion, tm, "HEAD_LINK1", "xtion_link"));
    tfs.push_back(PT2TF(X_0_base_odom, tm, "map", "odom_base_link"));
    tfs.push_back(PT2TF(X_base_xtion, tm, "odom_base_link", "odom_xtion_link"));

    mut.lock();
    msgs.push({msg, tfs});
    mut.unlock();
  }
private:
  ros::NodeHandle nh;
  ros::Publisher j_state_pub;
  tf2_ros::TransformBroadcaster tf_caster;

  struct RobotStateData
  {
    sensor_msgs::JointState js;
    std::vector<geometry_msgs::TransformStamped> tfs;
  };

  uint64_t seq;
  bool running;
  std::queue<RobotStateData> msgs;
  std::thread th;
  std::mutex mut;

  void publishThread()
  {
    ros::Rate rt(500);
    while(running)
    {
      while(msgs.size())
      {
        if(mut.try_lock())
        {
          const auto & msg = msgs.front();
          try
          {
            j_state_pub.publish(msg.js);
            tf_caster.sendTransform(msg.tfs);
          }
          catch(const ros::serialization::StreamOverrunException & e)
          {
            std::cerr << "EXCEPTION WHILE PUBLISHING STATE" << std::endl;
            std::cerr << e.what() << std::endl;
          }
          msgs.pop();
          mut.unlock();
        }
      }
      rt.sleep();
    }
  }
};
#else
struct RobotPublisherImpl
{
  RobotPublisherImpl(const std::string&) {}
  void stop() {}
  void update(const mc_rbdyn::Robot &) {}
};
#endif

RobotPublisher::RobotPublisher(const std::string & node_name)
: impl(new RobotPublisherImpl(node_name))
{
}

void RobotPublisher::stop()
{
  impl->stop();
}

void RobotPublisher::update(const mc_rbdyn::Robot & robot, const RTC::TimedPoint3D & p, const RTC::TimedOrientation3D & rpy)
{
  impl->update(robot, p, rpy);
}

}
