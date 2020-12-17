/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/RobotModule.h>

#include "api.h"

namespace mc_robots
{

struct MC_ROBOTS_DLLAPI JVRC1RobotModule : public mc_rbdyn::RobotModule
{
public:
  JVRC1RobotModule(bool fixed = false);
};

} // namespace mc_robots

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    names = {"JVRC1", "JVRC1Fixed"};
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & name)
  {
    if(name == "JVRC1")
    {
      return new mc_robots::JVRC1RobotModule(false);
    }
    else if(name == "JVRC1Fixed")
    {
      return new mc_robots::JVRC1RobotModule(true);
    }
    else
    {
      mc_rtc::log::error("JVRC1 module Cannot create an object of type {}", name);
      return nullptr;
    }
  }
}
