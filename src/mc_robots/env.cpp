/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "env.h"

#include <RBDyn/parsers/urdf.h>

#include <mc_rtc/logging.h>

#include <boost/filesystem.hpp>

#include <fstream>
namespace bfs = boost::filesystem;

namespace mc_robots
{

EnvRobotModule::EnvRobotModule(const std::string & env_path, const std::string & env_name, bool fixed)
: RobotModule(env_path, env_name)
{
  init(rbd::parsers::from_urdf_file(urdf_path, fixed));

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

} // namespace mc_robots
