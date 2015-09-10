#pragma once

#include <rtm/idl/BasicDataTypeSkel.h>
#include <memory>

namespace mc_rtc
{

struct JointPublisherImpl;

struct JointPublisher
{
public:
  JointPublisher(const std::string & node_name);

  void stop();

  void new_state(const RTC::TimedDoubleSeq & q);
private:
  std::shared_ptr<JointPublisherImpl> impl;
};

}
