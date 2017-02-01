#pragma once

#include <mc_rtc/config.h>
#include <mc_rtc/ros.h>

#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/Contact.h>

#include <Eigen/Geometry>

#include <geos/geom/Geometry.h>

#include <queue>
#include <thread>

#include <mc_control/api.h>

namespace mc_control
{

struct MC_CONTROL_DLLAPI MCSeqPublisher
{
public:
  MCSeqPublisher(const mc_rbdyn::Robots & robots);

  ~MCSeqPublisher();

  void publish_com(const Eigen::Vector3d & com);

  void publish_poly(const std::shared_ptr<geos::geom::Geometry> & geom);

  void publish_waypoint(const sva::PTransformd & X_waypoint);

  void set_contacts(const std::vector<mc_rbdyn::Contact> & contacts);

  /* Return the slam position of the contact being added */
  sva::PTransformd get_slam_contact();
private:
  const mc_rbdyn::Robots & robots;
  std::shared_ptr<ros::NodeHandle> nh;
  std::thread pub_thread;
  Eigen::Vector3d com;
  std::vector<Eigen::Vector3d> poly;
  sva::PTransformd X_waypoint;
  std::vector<mc_rbdyn::Contact> contacts;
  std::string slam_contact;
  sva::PTransformd X_slam_contact;

  bool running;
  void publication_thread();
};

}
