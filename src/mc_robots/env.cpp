#include "env.h"

#include <mc_rtc/logging.h>

#include <fstream>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

namespace mc_robots
{

EnvRobotModule::EnvRobotModule(const std::string & env_path, const std::string & env_name)
: RobotModule(env_path, env_name)
{
  std::ifstream ifs(urdf_path);
  if(ifs.is_open())
  {
    std::stringstream urdf;
    urdf << ifs.rdbuf();
    mc_rbdyn_urdf::URDFParserResult res = mc_rbdyn_urdf::rbdyn_from_urdf(urdf.str());
    mb = res.mb;
    mbc = res.mbc;
    mbg = res.mbg;
    boundsFromURDF(res.limits);
    _collisionTransforms = res.collision_tf;

    std::string convexPath = path + "/convex/" + name + "/";
    bfs::path p(convexPath);
    if(bfs::exists(p) && bfs::is_directory(p))
    {
      std::vector<bfs::path> files;
      std::copy(bfs::directory_iterator(p), bfs::directory_iterator(), std::back_inserter(files));
      for(const bfs::path & file : files)
      {
        size_t off = file.filename().string().rfind("-ch.txt");
        if(off != std::string::npos)
        {
          std::string name = file.filename().string();
          name.replace(off, 7, "");
          _convexHull[name] = std::pair<std::string, std::string>(name, file.string());
        }
      }
    }
  }
  else
  {
    LOG_ERROR("Could not load env model at " << urdf_path)
    throw("Could not open env model");
  }
}

}
