#include <mc_rtc/ros.h>
#include <mc_rtc/config.h>


#ifdef MC_RTC_HAS_ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

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

struct JointPublisherImpl
{
public:
  JointPublisherImpl(const std::string & node_name)
  : nh(ros_init(node_name)),
    pub(nh.advertise<sensor_msgs::JointState>("joint_states", 1)),
    seq(0), running(true), msgs(),
    th(std::bind(&JointPublisherImpl::publishThread, this))
  {
  }

  ~JointPublisherImpl()
  {
    stop();
  }

  void stop()
  {
    running = false;
    th.join();
  }
  void new_state(const RTC::TimedDoubleSeq & q)
  {
    sensor_msgs::JointState msg;
    msg.header.seq = ++seq;
    msg.header.stamp.sec = q.tm.sec;
    msg.header.stamp.nsec = q.tm.nsec;
    msg.header.frame_id = "";
    msg.name = REF_JOINT_ORDER;
    msg.position.reserve(REF_JOINT_ORDER.size());
    for(size_t i = 0; i < q.data.length(); ++i)
    {
      msg.position.push_back(q.data[i]);
    }
    msg.velocity.resize(0);
    msg.effort.resize(0);
    mut.lock();
    msgs.push(msg);
    mut.unlock();
  }
private:
  ros::NodeHandle nh;
  ros::Publisher pub;

  uint64_t seq;
  bool running;
  std::queue<sensor_msgs::JointState> msgs;
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
            pub.publish(msg);
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
struct JointPublisherImpl
{
  JointPublisherImpl(const std::string&) {}
  void stop() {}
  void new_state(const RTC::TimedDoubleSeq &) {}
};
#endif

JointPublisher::JointPublisher(const std::string & node_name)
: impl(new JointPublisherImpl(node_name))
{
}

void JointPublisher::stop()
{
  impl->stop();
}

void JointPublisher::new_state(const RTC::TimedDoubleSeq & q)
{
  impl->new_state(q);
}

}
