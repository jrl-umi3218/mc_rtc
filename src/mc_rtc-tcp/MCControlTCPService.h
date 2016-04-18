#pragma once

#include <memory>
#include <vector>

#include <mc_control/mc_global_controller.h>

/* pimpl pattern */
struct MCControlTCPServiceImpl;

/*! This class implements service call for the MCControlTCP executable.
 * Following mc_rtc principle, it is disabled when ROS is not available as the
 * executable is started or if ROS was not available at build time */
class MCControlTCPService
{
public:
  MCControlTCPService(mc_control::MCGlobalController & controller);

  ~MCControlTCPService();
private:
  std::unique_ptr<MCControlTCPServiceImpl> impl;
};
