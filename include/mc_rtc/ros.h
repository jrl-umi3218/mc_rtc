#pragma once

#include <memory>
#include <rtm/idl/ExtendedDataTypesSkel.h>

namespace mc_rbdyn
{
  struct Robot;
}

namespace mc_rtc
{

struct RobotPublisherImpl;

struct RobotPublisher
{
public:
  RobotPublisher(const std::string & node_name);

  void stop();

  void update(const mc_rbdyn::Robot & robot, const RTC::TimedPoint3D & p, const RTC::TimedOrientation3D & rpy);
private:
  std::shared_ptr<RobotPublisherImpl> impl;
};

}
