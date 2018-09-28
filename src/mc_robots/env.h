#pragma once

#include <mc_rbdyn/RobotModule.h>
#include <mc_rtc/config.h>

#include <mc_rbdyn_urdf/urdf.h>

#include "api.h"

namespace mc_robots
{

struct MC_ROBOTS_DLLAPI EnvRobotModule : public mc_rbdyn::RobotModule
{
public:
  EnvRobotModule(const std::string & env_path, const std::string & env_name, bool fixed);
};

} // namespace mc_robots

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    names = {"env", "object"};
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & type,
                                                  const std::string & path,
                                                  const std::string & name)
  {
    bool fixed = type == "env";
    if(path == "@MC_ENV_DESCRIPTION@")
    {
      return new mc_robots::EnvRobotModule(mc_rtc::MC_ENV_DESCRIPTION_PATH, name, fixed);
    }
    return new mc_robots::EnvRobotModule(path, name, fixed);
  }
}
