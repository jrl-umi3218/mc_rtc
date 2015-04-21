#include <mc_robots/env.h>

#include <fstream>

namespace mc_robots
{

//FIXME This path should be passed as a parameters
const std::string GroundRobotModule::path = "/home/gergondet/devel-src/mcp/mcp_ws/src/mc_ros/mc_env_description";

GroundRobotModule::GroundRobotModule()
{
  std::string urdfPath = path + "/urdf/ground.urdf";
  std::ifstream ifs(urdfPath);
  std::stringstream urdf;
  urdf << ifs.rdbuf();
  mc_rbdyn_urdf::URDFParserResult res = mc_rbdyn_urdf::rbdyn_from_urdf(urdf.str(), false, {});
  mb = res.mb;
  mbc = res.mbc;
  mbg = res.mbg;
}

const std::map<std::string, std::pair<std::string, std::string> > & GroundRobotModule::convexHull() const
{
  std::string convexPath = path + "/convex/ground/";
  std::map<std::string, std::pair<std::string, std::string> > res;
  res["ground"] = std::pair<std::string, std::string>("ground", convexPath + "/ground-ch.txt");
  const_cast<GroundRobotModule*>(this)->_convexHull = res;
  return _convexHull;
}

}
