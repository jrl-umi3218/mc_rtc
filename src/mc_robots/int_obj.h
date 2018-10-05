#pragma once

#include <mc_rbdyn/RobotModule.h>
#include <mc_rtc/config.h>

#include <mc_rbdyn_urdf/urdf.h>

#include "api.h"

namespace mc_robots
{

struct MC_ROBOTS_DLLAPI IntObjRobotModule : public mc_rbdyn::RobotModule
{
public:
  IntObjRobotModule(const std::string & env_path, const std::string & env_name);
};

} // namespace mc_robots

extern "C"
{
  ROBOT_MODULE_COMMON("int_obj")
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string &,
                                                  const std::string & path,
                                                  const std::string & name)
  {
    if(path == "@MC_ENV_DESCRIPTION@")
    {
      return new mc_robots::IntObjRobotModule(mc_rtc::MC_ENV_DESCRIPTION_PATH, name);
    }
    return new mc_robots::IntObjRobotModule(path, name);
  }
}
