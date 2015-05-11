#ifndef _H_MCROBOTSGROUND_H_
#define _H_MCROBOTSGROUND_H_

#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn_urdf/urdf.h>

namespace mc_robots
{

struct EnvRobotModule : public mc_rbdyn::RobotModule
{
public:
  EnvRobotModule(const std::string & env_path, const std::string & env_name);

  virtual const std::map<std::string, std::pair<std::string, std::string> > & convexHull() const;
};

}

#endif
