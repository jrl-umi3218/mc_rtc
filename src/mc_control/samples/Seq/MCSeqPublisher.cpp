#include "MCSeqPublisher.h"

#include <geos/geom/Polygon.h>
#include <geos/geom/LinearRing.h>
#include <geos/geom/CoordinateSequence.h>

#ifdef MC_RTC_HAS_ROS
#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#endif

#include <array>
#include <functional>

namespace mc_control
{

MCSeqPublisher::MCSeqPublisher(const mc_rbdyn::Robots & robots)
: robots(robots),
  nh(mc_rtc::ROSBridge::get_node_handle()),
  pub_thread(std::bind(&MCSeqPublisher::publication_thread, this)),
  com(Eigen::Vector3d::Zero()), poly(),
  X_waypoint(sva::PTransformd::Identity()), contacts(),
  slam_contact(""), X_slam_contact(sva::PTransformd::Identity()),
  running(true)
{
}

MCSeqPublisher::~MCSeqPublisher()
{
  running = false;
  pub_thread.join();
}

void MCSeqPublisher::publish_com(const Eigen::Vector3d & com)
{
  this->com = com;
}

void MCSeqPublisher::publish_poly(const std::shared_ptr<geos::geom::Geometry> & geom)
{
  geos::geom::Polygon * polyIn = dynamic_cast<geos::geom::Polygon *>(geom.get());
  if(polyIn)
  {
    poly.clear();
    const geos::geom::CoordinateSequence * seq = polyIn->getExteriorRing()->getCoordinates();
    for(size_t i = 0; i < seq->size(); ++i)
    {
      const geos::geom::Coordinate & p = seq->getAt(i);
      poly.emplace_back(p.x, p.y, 0);
    }
  }
}

void MCSeqPublisher::publish_waypoint(const sva::PTransformd & X_waypoint)
{
  this->X_waypoint = X_waypoint;
}

void MCSeqPublisher::set_contacts(const std::vector<mc_rbdyn::Contact> & cs)
{
  /* Skip new contact determination for first contact */
  if(contacts.size() != 0)
  {
    std::string new_slam_contact = "";
    for(const auto & c : cs)
    {
      bool new_contact = true;
      for(const auto & pc : contacts)
      {
        if(pc == c)
        {
          new_contact = false;
          break;
        }
      }
      if(new_contact)
      {
        std::stringstream ss;
        ss << "pxtools_contact_" << c.r1Surface()->name() << "_" << c.r2Surface()->name();
        new_slam_contact = ss.str();
        std::cout << "Tracking new slam contact " << new_slam_contact << std::endl;
      }
    }
    slam_contact = new_slam_contact;
  }
  this->contacts = cs;
}

sva::PTransformd MCSeqPublisher::get_slam_contact()
{
  return X_slam_contact;
}

#ifdef MC_RTC_HAS_ROS
inline visualization_msgs::Marker com_marker(const Eigen::Vector3d & com, const std_msgs::Header & header)
{
  visualization_msgs::Marker msg;
  msg.header = header;
  msg.ns = "robot";
  msg.id = static_cast<int>(std::hash<std::string>()("com_marker"));
  msg.type = visualization_msgs::Marker::SPHERE;
  msg.action = 0;
  msg.pose.orientation.w = 1.0;
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = 0.0;
  msg.pose.position.x = com.x();
  msg.pose.position.y = com.y();
  msg.pose.position.z = 0;//com.z();
  msg.scale.x = 0.05;
  msg.scale.y = 0.05;
  msg.scale.z = 0.05;
  msg.color.r = 1.0;
  msg.color.g = 0.0;
  msg.color.b = 0.0;
  msg.color.a = 1.0;
  msg.lifetime = ros::Duration(1);
  return msg;
}

inline geometry_msgs::PolygonStamped poly_msg(const std::vector<Eigen::Vector3d> & poly, const std_msgs::Header & header)
{
  geometry_msgs::PolygonStamped msg;
  msg.header = header;
  msg.polygon.points.reserve(poly.size());
  for(const auto & p : poly)
  {
    geometry_msgs::Point32 pt;
    pt.x = static_cast<float>(p.x());
    pt.y = static_cast<float>(p.y());
    pt.z = static_cast<float>(p.z());
    msg.polygon.points.push_back(pt);
  }
  return msg;
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

inline void TF2PT(const geometry_msgs::TransformStamped & tf, sva::PTransformd & X)
{
  X = sva::PTransformd(Eigen::Quaterniond(
                                          tf.transform.rotation.w,
                                          tf.transform.rotation.x,
                                          tf.transform.rotation.y,
                                          tf.transform.rotation.z).inverse(),
                       Eigen::Vector3d(
                                       tf.transform.translation.x,
                                       tf.transform.translation.y,
                                       tf.transform.translation.z));
}

inline visualization_msgs::Marker marker(const std::string & name, const sva::PTransformd & X, const std_msgs::Header & header, std::array<float, 4> rgba)
{
  visualization_msgs::Marker msg;
  msg.header = header;
  Eigen::Quaterniond ori(X.rotation());
  msg.ns = "robot";
  msg.id = static_cast<int>(std::hash<std::string>()(name));
  msg.type = visualization_msgs::Marker::SPHERE;
  msg.action = 0;
  msg.pose.orientation.w = ori.w();
  msg.pose.orientation.x = ori.x();
  msg.pose.orientation.y = ori.y();
  msg.pose.orientation.z = ori.z();
  msg.pose.position.x = X.translation().x();
  msg.pose.position.y = X.translation().y();
  msg.pose.position.z = X.translation().z();
  msg.scale.x = 0.05;
  msg.scale.y = 0.05;
  msg.scale.z = 0.05;
  msg.color.r = rgba[0];
  msg.color.g = rgba[1];
  msg.color.b = rgba[2];
  msg.color.a = rgba[3];
  msg.lifetime = ros::Duration(0.5);
  msg.text = name;
  return msg;
}

inline visualization_msgs::Marker contact_marker(const mc_rbdyn::Robots & robots, const mc_rbdyn::Contact & contact, const std_msgs::Header & header, std::vector<geometry_msgs::TransformStamped> & tfs)
{
  auto X_0_c = contact.X_0_r1s(robots);
  std::array<float, 4> rgba = {0.0, 1.0, 0.0, 1.0};
  visualization_msgs::Marker msg = marker(contact.toStr(), X_0_c, header, rgba);
  std::stringstream ss;
  ss << "contact_" << contact.r1Surface()->name() << "_" << contact.r2Surface()->name();
  /* TF expressed in robot's frame */
  tfs.push_back(PT2TF(X_0_c, header.stamp, header.frame_id, ss.str()));
  /* TF expressed in vision-based object's frame */
  std::stringstream ss2;
  ss2 << "pxtools_" << ss.str();
  tfs.push_back(PT2TF(X_0_c, header.stamp, "pxtools_relative", ss2.str()));
  return msg;
}

void MCSeqPublisher::publication_thread()
{
  if(nh)
  {
    std::array<float, 4> rgba = {1.0, .0, 0.0, 1.0};
    ros::Rate rt(30);
    tf2_ros::TransformBroadcaster tf_caster;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    std::vector<geometry_msgs::TransformStamped> tfs;
    ros::Publisher com_pub = nh->advertise<visualization_msgs::Marker>("com_marker", 1);
    ros::Publisher contact_pub = nh->advertise<visualization_msgs::Marker>("contact_makers", 1);
    ros::Publisher poly_pub = nh->advertise<geometry_msgs::PolygonStamped>("stability_polygon", 10);
    std_msgs::Header header;
    header.seq = 0;
    while(running)
    {
      tfs.clear();
      ++header.seq;
      header.stamp = ros::Time::now();
      header.frame_id = "robot_map";
      com_pub.publish(com_marker(com, header));
      poly_pub.publish(poly_msg(poly, header));
      for(const auto & contact : contacts)
      {
        contact_pub.publish(contact_marker(robots, contact, header, tfs));
      }
      if(X_waypoint != sva::PTransformd::Identity())
      {
        contact_pub.publish(marker("WPT", X_waypoint, header, rgba));
        tfs.push_back(PT2TF(X_waypoint, header.stamp, header.frame_id, "WPT"));
      }
      tf_caster.sendTransform(tfs);
      ros::spinOnce();
      if(slam_contact != "")
      {
        try
        {
          TF2PT(tf_buffer.lookupTransform("robot_map", slam_contact, ros::Time(0)), X_slam_contact);
        }
        catch(...)
        {
        }
      }
      rt.sleep();
    }
  }
}
#else
void MCSeqPublisher::publication_thread()
{
}
#endif

}
