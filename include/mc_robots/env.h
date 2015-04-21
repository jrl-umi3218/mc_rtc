#ifndef _H_MCROBOTSGROUND_H_
#define _H_MCROBOTSGROUND_H_

#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn_urdf/urdf.h>

namespace mc_robots
{

struct GroundRobotModule : public mc_rbdyn::RobotModule
{
public:
  //FIXME This path should be passed as a parameters
  static const std::string path;
public:
  GroundRobotModule();
  /* FIXME implemented in the python but not really used in the C++ so far */
  /* mb,mbc,mbg robot(const bool & fixed = false); */

  virtual const std::map<std::string, std::pair<std::string, std::string> > & convexHull() const;
};

}

#endif
