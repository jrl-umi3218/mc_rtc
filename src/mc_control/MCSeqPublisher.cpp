#include <mc_control/MCSeqPublisher.h>

#include <geos/geom/Polygon.h>
#include <geos/geom/LinearRing.h>
#include <geos/geom/CoordinateSequence.h>

#ifdef MC_RTC_HAS_ROS
#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>
#endif

#include <functional>

namespace mc_control
{

MCSeqPublisher::MCSeqPublisher()
: nh(mc_rtc::ROSBridge::get_node_handle()),
  pub_thread(std::bind(&MCSeqPublisher::publication_thread, this)),
  com(Eigen::Vector3d::Zero()),
  running(true)
{
}

MCSeqPublisher::~MCSeqPublisher()
{
  running = false;
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
  msg.scale.x = 0.1;
  msg.scale.y = 0.1;
  msg.scale.z = 0.1;
  msg.color.r = 1.0;
  msg.color.g = 0.0;
  msg.color.b = 0.0;
  msg.color.a = 1.0;
  msg.lifetime = ros::Duration(0);
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
    pt.x = p.x();
    pt.y = p.y();
    pt.z = p.z();
    msg.polygon.points.push_back(pt);
  }
  return msg;
}

void MCSeqPublisher::publication_thread()
{
  if(nh)
  {
    ros::Rate rt(60);
    ros::Publisher com_pub = nh->advertise<visualization_msgs::Marker>("com_marker", 1);
    ros::Publisher poly_pub = nh->advertise<geometry_msgs::PolygonStamped>("stability_polygon", 1);
    std_msgs::Header header;
    header.seq = 0;
    while(running)
    {
      ++header.seq;
      header.stamp = ros::Time::now();
      header.frame_id = "map";
      com_pub.publish(com_marker(com, header));
      poly_pub.publish(poly_msg(poly, header));
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
