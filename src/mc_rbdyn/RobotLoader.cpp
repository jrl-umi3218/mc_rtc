/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/RobotLoader.h>

#include <mc_rtc/Configuration.h>

#include <boost/filesystem.hpp>
#include <boost/range/adaptors.hpp>
namespace bfs = boost::filesystem;

std::unique_ptr<mc_rtc::ObjectLoader<mc_rbdyn::RobotModule>> mc_rbdyn::RobotLoader::robot_loader;
bool mc_rbdyn::RobotLoader::enable_sandbox_ = false;
bool mc_rbdyn::RobotLoader::verbose_ = false;
std::mutex mc_rbdyn::RobotLoader::mtx{};
std::map<std::string, std::vector<std::string>> mc_rbdyn::RobotLoader::aliases{};

namespace
{

void handle_aliases_dir(const bfs::path & dir)
{
  if(!bfs::exists(dir) || !bfs::is_directory(dir))
  {
    return;
  }
  bfs::directory_iterator dit(dir), endit;
  std::vector<bfs::path> drange;
  std::copy(dit, endit, std::back_inserter(drange));
  for(const auto & p : drange)
  {
    const auto & extension = bfs::extension(p);
    if(extension == ".yml" || extension == ".json" || extension == ".yaml")
    {
      mc_rbdyn::RobotLoader::load_aliases(p.string());
    }
  }
}

} // namespace

void mc_rbdyn::RobotLoader::load_aliases(const std::string & fname)
{
  if(verbose_)
  {
    LOG_INFO("[RobotLoader] Loading aliases from " << fname)
  }
  mc_rtc::Configuration data(fname);
  try
  {
    std::map<std::string, mc_rtc::Configuration> new_aliases = data;
    for(const auto & a : new_aliases)
    {
      if(robot_loader->has_object(a.first))
      {
        LOG_WARNING("Aliases declaration " << a.first << " in " << fname
                                           << " would shadow library declaration, discarding this alias")
        continue;
      }
      else if(aliases.count(a.first))
      {
        LOG_WARNING("Aliases " << a.first << " was already declared, new declaration from " << fname << " will prevail")
      }
      if(a.second.size())
      {
        aliases[a.first] = a.second;
      }
      else
      {
        aliases[a.first] = {static_cast<std::string>(a.second)};
      }
      if(verbose_)
      {
        LOG_INFO("New alias " << a.first << ": " << data(a.first).dump(true, true))
      }
    }
  }
  catch(mc_rtc::Configuration::Exception & exc)
  {
    LOG_ERROR("Loading of RobotModule aliases file: " << fname << " failed")
    LOG_WARNING(exc.what())
    exc.silence();
  }
}

std::vector<std::string> mc_rbdyn::RobotLoader::available_robots()
{
  std::lock_guard<std::mutex> guard{mtx};
  init();
  auto ret = robot_loader->objects();
  for(const auto & a : aliases)
  {
    ret.push_back(a.first);
  }
  return ret;
}

void mc_rbdyn::RobotLoader::update_robot_module_path(const std::vector<std::string> & paths)
{
  std::lock_guard<std::mutex> guard{mtx};
  init();
  robot_loader->load_libraries(paths);
  for(const auto & p : paths)
  {
    handle_aliases_dir(bfs::path(p) / "aliases");
  }
}

void mc_rbdyn::RobotLoader::init(bool skip_default_path)
{
  if(!robot_loader)
  {
    try
    {
      std::vector<std::string> default_path = {};
      if(!skip_default_path)
      {
        default_path.push_back(mc_rtc::MC_ROBOTS_INSTALL_PREFIX);
      }
      robot_loader.reset(new mc_rtc::ObjectLoader<mc_rbdyn::RobotModule>("MC_RTC_ROBOT_MODULE", default_path,
                                                                         enable_sandbox_, verbose_));
      for(const auto & p : default_path)
      {
        handle_aliases_dir(bfs::path(p) / "aliases");
      }
#ifndef WIN32
      handle_aliases_dir(bfs::path(std::getenv("HOME")) / ".config/mc_rtc/aliases/");
#else
      // Should work for Windows Vista and up
      handle_aliases_dir(bfs::path(std::getenv("APPDATA")) / "mc_rtc/aliases/");
#endif
    }
    catch(const mc_rtc::LoaderException & exc)
    {
      LOG_ERROR("Failed to initialize RobotLoader: " << exc.what())
      throw(exc);
    }
  }
}
