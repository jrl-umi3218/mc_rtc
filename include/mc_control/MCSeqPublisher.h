#pragma once

#include <mc_rtc/config.h>
#include <mc_rtc/ros.h>

#include <Eigen/Geometry>

#include <geos/geom/Geometry.h>

#include <queue>
#include <thread>

namespace mc_control
{

struct MCSeqPublisher
{
public:
  MCSeqPublisher();

  ~MCSeqPublisher();

  void publish_com(const Eigen::Vector3d & com);

  void publish_poly(const std::shared_ptr<geos::geom::Geometry> & geom);
private:
  std::shared_ptr<ros::NodeHandle> nh;
  std::thread pub_thread;
  Eigen::Vector3d com;
  std::vector<Eigen::Vector3d> poly;

  bool running;
  void publication_thread();
};

}
