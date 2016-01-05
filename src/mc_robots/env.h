#pragma once

#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn_urdf/urdf.h>

#include "api.h"

namespace mc_robots
{

struct MC_ROBOTS_DLLAPI EnvRobotModule : public mc_rbdyn::RobotModule
{
public:
  EnvRobotModule(const std::string & env_path, const std::string & env_name);

  virtual const std::map<std::string, std::pair<std::string, std::string> > & convexHull() const;
};

}

ROBOT_MODULE_CANONIC_CONSTRUCTOR("env", mc_robots::EnvRobotModule)
