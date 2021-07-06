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
  if(!fixed)
  {
    _bodySensors.emplace_back("FloatingBase", mb.body(0).name(), sva::PTransformd::Identity());
  }
}

} // namespace mc_robots

#ifndef MC_RTC_BUILD_STATIC

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

#else

#  include <mc_rbdyn/RobotLoader.h>

namespace
{

static auto registered = []() {
  using fn_t = std::function<mc_robots::EnvRobotModule *(const std::string &, const std::string &)>;
  auto make_callback = [](bool fixed) -> fn_t {
    return [fixed](const std::string & path, const std::string & name) {
      if(path == "@MC_ENV_DESCRIPTION_PATH@")
      {
        return new mc_robots::EnvRobotModule(mc_rtc::MC_ENV_DESCRIPTION_PATH, name, fixed);
      }
      return new mc_robots::EnvRobotModule(path, name, fixed);
    };
  };
  mc_rbdyn::RobotLoader::register_object("env", make_callback(true));
  mc_rbdyn::RobotLoader::register_object("object", make_callback(false));
  return true;
}();
} // namespace

#endif
