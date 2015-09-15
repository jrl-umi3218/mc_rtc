#pragma once

#include <memory>

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

  void update(const mc_rbdyn::Robot & robot);
private:
  std::shared_ptr<RobotPublisherImpl> impl;
};

}
