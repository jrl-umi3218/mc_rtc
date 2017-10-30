#pragma once

#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn_urdf/urdf.h>

#include <mc_rtc/config.h>

#include "api.h"

namespace mc_robots
{

struct MC_ROBOTS_DLLAPI EnvRobotModule : public mc_rbdyn::RobotModule
{
public:
  EnvRobotModule(const std::string & env_path, const std::string & env_name);
};

}

extern "C"
{
  ROBOT_MODULE_COMMON("env")
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string&,
                                                  const std::string & path,
                                                  const std::string & name)
  {
    if(path == "@MC_ENV_DESCRIPTION@")
    {
      return new mc_robots::EnvRobotModule(mc_rtc::MC_ENV_DESCRIPTION_PATH, name);
    }
    return new mc_robots::EnvRobotModule(path, name);
  }
}
