#include <mc_robots/env.h>

#include <fstream>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

namespace mc_robots
{

EnvRobotModule::EnvRobotModule(const std::string & env_path, const std::string & env_name)
: RobotModule(env_path, env_name)
{
  std::string urdfPath = path + "/urdf/" + name + ".urdf";
  std::ifstream ifs(urdfPath);
  std::stringstream urdf;
  urdf << ifs.rdbuf();
  mc_rbdyn_urdf::URDFParserResult res = mc_rbdyn_urdf::rbdyn_from_urdf(urdf.str());
  mb = res.mb;
  mbc = res.mbc;
  mbg = res.mbg;
  _collisionTransforms = res.collision_tf;
}

const std::map<std::string, std::pair<std::string, std::string> > & EnvRobotModule::convexHull() const
{
  std::string convexPath = path + "/convex/" + name + "/";
  std::map<std::string, std::pair<std::string, std::string> > res;

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
        res[name] = std::pair<std::string, std::string>(name, file.string());
      }
    }
  }

  const_cast<EnvRobotModule*>(this)->_convexHull = res;
  return _convexHull;
}

}
