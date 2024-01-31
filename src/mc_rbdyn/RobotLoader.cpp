/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/RobotLoader.h>

#include <mc_rtc/Configuration.h>
#include <mc_rtc/path.h>

#include <boost/filesystem.hpp>
#include <boost/range/adaptors.hpp>
namespace bfs = boost::filesystem;

std::unique_ptr<mc_rtc::ObjectLoader<mc_rbdyn::RobotModule>> mc_rbdyn::RobotLoader::robot_loader;
bool mc_rbdyn::RobotLoader::verbose_ = false;
std::mutex mc_rbdyn::RobotLoader::mtx{};
std::map<std::string, std::vector<std::string>> mc_rbdyn::RobotLoader::aliases{};

namespace
{

void handle_aliases_dir(const bfs::path & dir)
{
  if(!bfs::exists(dir) || !bfs::is_directory(dir)) { return; }
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

namespace mc_rbdyn
{

void RobotLoader::load_aliases(const std::string & fname)
{
  if(verbose_) { mc_rtc::log::info("[RobotLoader] Loading aliases from {}", fname); }
  mc_rtc::Configuration data(fname);
  try
  {
    std::map<std::string, mc_rtc::Configuration> new_aliases = data;
    for(const auto & a : new_aliases)
    {
      if(robot_loader->has_object(a.first))
      {
        mc_rtc::log::warning("Aliases declaration {} in {} would shadow library declaration, discarding this alias",
                             a.first, fname);
        continue;
      }
      else if(aliases.count(a.first))
      {
        mc_rtc::log::warning("Aliases {} was already declared, new declaration from {} will prevail", a.first, fname);
      }
      if(a.second.size()) { aliases[a.first] = a.second; }
      else { aliases[a.first] = {static_cast<std::string>(a.second)}; }
      if(verbose_) { mc_rtc::log::info("New alias {}: {}", a.first, data(a.first).dump(true, true)); }
    }
  }
  catch(mc_rtc::Configuration::Exception & exc)
  {
    mc_rtc::log::error("Loading of RobotModule aliases file: {} failed", fname);
    mc_rtc::log::warning(exc.what());
    exc.silence();
  }
}

std::vector<std::string> RobotLoader::available_robots()
{
  std::lock_guard<std::mutex> guard{mtx};
  init();
  auto ret = robot_loader->objects();
  for(const auto & a : aliases) { ret.push_back(a.first); }
  return ret;
}

void RobotLoader::update_robot_module_path(const std::vector<std::string> & paths)
{
  std::lock_guard<std::mutex> guard{mtx};
  init();
  robot_loader->load_libraries(paths);
  for(const auto & p : paths) { handle_aliases_dir(bfs::path(p) / "aliases"); }
}

void RobotLoader::init(bool skip_default_path)
{
  if(!robot_loader)
  {
    try
    {
      std::vector<std::string> default_path = {};
      if(!skip_default_path) { default_path.push_back(mc_rtc::MC_ROBOTS_INSTALL_PREFIX); }
      robot_loader.reset(new mc_rtc::ObjectLoader<RobotModule>("MC_RTC_ROBOT_MODULE", default_path, verbose_));
      for(const auto & p : default_path) { handle_aliases_dir(bfs::path(p) / "aliases"); }
      if(!skip_default_path) { handle_aliases_dir(mc_rtc::user_config_directory_path("aliases")); }
    }
    catch(const mc_rtc::LoaderException & exc)
    {
      mc_rtc::log::error("Failed to initialize RobotLoader: {}", exc.what());
      throw(exc);
    }
  }
}

RobotModulePtr RobotLoader::get_robot_module(const std::vector<std::string> & args)
{
  if(args.size() == 1) { return get_robot_module(args[0]); }
  if(args.size() == 2) { return get_robot_module(args[0], args[1]); }
  if(args.size() == 3) { return get_robot_module(args[0], args[1], args[2]); }
  mc_rtc::log::error_and_throw<mc_rtc::LoaderException>(
      "RobotLoader dynamic arguments should have 1 to 3 arguments but {} were provided", args.size());
}

} // namespace mc_rbdyn
