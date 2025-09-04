/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/mc_global_controller.h>

#include <mc_observers/ObserverLoader.h>

#include <mc_rbdyn/RobotLoader.h>

#include <mc_rtc/ConfigurationHelpers.h>
#include <mc_rtc/io_utils.h>
#include <mc_rtc/path.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

/* Implementation file for mc_control::MCGlobalController::Configuration */

namespace mc_control
{

MCGlobalController::GlobalConfiguration::GlobalConfiguration(const std::string & conf,
                                                             std::shared_ptr<mc_rbdyn::RobotModule> rm,
                                                             bool conf_only)
{
  if(!conf_only)
  {
    // Load default configuration file
    std::string globalPath(mc_rtc::CONF_PATH);
    if(bfs::exists(globalPath))
    {
      mc_rtc::log::info("Loading default global configuration {}", globalPath);
      config.load(globalPath);
    }

    bfs::path config_path = mc_rtc::user_config_directory_path("mc_rtc.conf");
    // Load user's local configuration if it exists
    if(!bfs::exists(config_path)) { config_path.replace_extension(".yaml"); }
    if(bfs::exists(config_path))
    {
      mc_rtc::log::info("Loading additional global configuration {}", config_path.string());
      config.load(config_path.string());
    }
  }
  // Load extra configuration
  if(bfs::exists(conf))
  {
    mc_rtc::log::info("Loading additional global configuration {}", conf);
    config.load(conf);
  }
  else if(conf_only) { mc_rtc::log::error_and_throw("Required to load {} only but this is not available", conf); }

  ///////////////////////
  //  General options  //
  ///////////////////////
  config("VerboseLoader", verbose_loader);
  config("Timestep", timestep);

  //////////////
  //  Robots  //
  //////////////
  mc_rbdyn::RobotLoader::set_verbosity(verbose_loader);
  config("RobotModulePaths", robot_module_paths);
  if(config("ClearRobotModulePath", false)) { mc_rbdyn::RobotLoader::clear(); }
  if(robot_module_paths.size())
  {
    try
    {
      mc_rbdyn::RobotLoader::update_robot_module_path(robot_module_paths);
    }
    catch(const mc_rtc::LoaderException & exc)
    {
      mc_rtc::log::error_and_throw("Failed to update robot module path(s)");
    }
  }
  if(rm) { main_robot_module = rm; }
  else
  {
    auto main_robot_params = [&]() -> std::vector<std::string>
    {
      auto main_robot_cfg = config.find("MainRobot");
      if(!main_robot_cfg) { return {"JVRC1"}; }
      if(main_robot_cfg->isArray()) { return main_robot_cfg->operator std::vector<std::string>(); }
      if(main_robot_cfg->isObject())
      {
        auto module_cfg = (*main_robot_cfg)("module");
        if(module_cfg.isArray()) { return module_cfg.operator std::vector<std::string>(); }
        return {module_cfg.operator std::string()};
      }
      return {main_robot_cfg->operator std::string()};
    }();
    if(!mc_rbdyn::RobotLoader::has_robot(main_robot_params[0]))
    {
      mc_rtc::log::error_and_throw("No loadable robot with module {}", main_robot_params[0]);
    }
    try
    {
      main_robot_module = mc_rbdyn::RobotLoader::get_robot_module(main_robot_params);
    }
    catch(const mc_rtc::LoaderException & exc)
    {
      mc_rtc::log::error_and_throw(
          "[mc_rtc::LoaderException] Failed to create [{}] to use as a main robot, exception: {}",
          mc_rtc::io::to_string(main_robot_params), exc.what());
    }
    catch(const std::exception & exc)
    {
      mc_rtc::log::error_and_throw("[std::exception] Failed to create [{}] to use as a main robot, exception: {}",
                                   mc_rtc::io::to_string(main_robot_params), exc.what());
    }
    catch(...)
    {
      mc_rtc::log::error_and_throw("[General exception] Failed to create [{}] to use as a main robot",
                                   mc_rtc::io::to_string(main_robot_params));
    }
  }
  main_robot_module->expand_stance();
  if(main_robot_module->ref_joint_order().empty()) { main_robot_module->make_default_ref_joint_order(); }

  /////////////////
  //  Observers  //
  /////////////////
  mc_observers::ObserverLoader::set_verbosity(verbose_loader);
  config("ObserverModulePaths", observer_module_paths);
  if(config("ClearObserverModulePath", false)) { mc_observers::ObserverLoader::clear(); }
  if(!observer_module_paths.empty())
  {
    try
    {
      mc_observers::ObserverLoader::update_module_path(observer_module_paths);
    }
    catch(const mc_rtc::LoaderException & exc)
    {
      mc_rtc::log::error_and_throw("Failed to update observer module path(s)");
    }
  }

  ///////////////
  //  Plugins  //
  ///////////////
  config("GlobalPluginPaths", global_plugin_paths);
  if(!config("ClearGlobalPluginPath", false))
  {
    global_plugin_paths.insert(global_plugin_paths.begin(), mc_rtc::MC_PLUGINS_INSTALL_PREFIX);
  }
  for(const auto & p : global_plugin_paths)
  {
    auto autoload_path = bfs::path(p) / "autoload";
    if(bfs::exists(autoload_path) && bfs::is_directory(autoload_path))
    {
      bfs::directory_iterator dit(autoload_path), endit;
      std::vector<bfs::path> drange;
      std::copy(dit, endit, std::back_inserter(drange));
      for(const auto & p : drange)
      {
        std::ifstream ifs(p.string());
        std::stringstream ss;
        ss << ifs.rdbuf();
        auto plugin = [&ss]()
        {
          auto out = ss.str();
          out.erase(std::find_if(out.rbegin(), out.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base(),
                    out.end());
          return out;
        }();
        if(std::find(global_plugins.begin(), global_plugins.end(), plugin) == global_plugins.end())
        {
          global_plugins.push_back(plugin);
          global_plugins_autoload.push_back(global_plugins.back());
        }
      }
    }
  }
  std::vector<std::string> plugins = mc_rtc::fromVectorOrElement<std::string>(config, "Plugins", {});
  for(const auto & p : plugins)
  {
    if(std::find(global_plugins.begin(), global_plugins.end(), p) == global_plugins.end())
    {
      global_plugins.push_back(p);
    }
  }

  ///////////////////
  //  Controllers  //
  ///////////////////
  config("ControllerModulePaths", controller_module_paths);
  if(!config("ClearControllerModulePath", false))
  {
    controller_module_paths.insert(controller_module_paths.begin(), mc_rtc::MC_CONTROLLER_INSTALL_PREFIX);
  }
  enabled_controllers = mc_rtc::fromVectorOrElement<std::string>(config, "Enabled", {});
  if(enabled_controllers.size()) { initial_controller = enabled_controllers[0]; }
  else { mc_rtc::log::error_and_throw("Enabled entry in mc_rtc must contain at least one controller name"); }
  config("Default", initial_controller);
  config("IncludeHalfSitController", include_halfsit_controller);

  ////////////////////
  // Initialization //
  ////////////////////
  config("InitAttitudeFromSensor", init_attitude_from_sensor);
  config("InitAttitudeSensor", init_attitude_sensor);

  ///////////////
  //  Logging  //
  ///////////////
  config("Log", enable_log);
  {
    std::string log_policy_str = "non-threaded";
    config("LogPolicy", log_policy_str);
    if(log_policy_str == "threaded") { log_policy = mc_rtc::Logger::Policy::THREADED; }
    else if(log_policy_str == "non-threaded") { log_policy = mc_rtc::Logger::Policy::NON_THREADED; }
    else
    {
      mc_rtc::log::warning("Unrecognized LogPolicy entry, will default to non-threaded");
      log_policy = mc_rtc::Logger::Policy::NON_THREADED;
    }
  }
  log_directory = bfs::temp_directory_path().string();
  {
    std::string v = "";
    config("LogDirectory", v);
    if(v.size()) { log_directory = v; }
  }
  config("LogTemplate", log_template);

  /////////////////////////
  //  GUI server options //
  /////////////////////////
  if(auto gui_config = config.find("GUIServer"))
  {
    enable_gui_server = (*gui_config)("Enable", false);
    gui_server_configuration.load(*gui_config);
  }
  else { enable_gui_server = false; }
}

namespace
{

bfs::path conf_or_yaml(bfs::path in)
{
  if(bfs::exists(in)) { return in; }
  in.replace_extension(".yaml");
  if(bfs::exists(in)) { return in; }
  in.replace_extension(".yml");
  return in;
}

/** Load a single configuration */
inline void load_config(const std::string & desc,
                        const std::string name,
                        const std::vector<std::string> & search_path,
                        const bfs::path & user_path,
                        std::unordered_map<std::string, mc_rtc::Configuration> & configs,
                        const mc_rtc::Configuration & default_config = {},
                        const std::initializer_list<const char *> & filter = {},
                        const bfs::path & search_path_suffix = bfs::path("etc"))
{
  mc_rtc::Configuration c;
  c.load(default_config);
  for(const auto & k : filter) { c.remove(k); }
  for(const auto & p : search_path)
  {
    bfs::path global = conf_or_yaml(bfs::path(p) / search_path_suffix / (name + ".conf"));
    if(bfs::exists(global))
    {
      mc_rtc::log::info("Loading additional {} configuration: {}", desc, global.string());
      c.load(global.string());
    }
  }
  bfs::path local = conf_or_yaml(user_path / (name + ".conf"));
  if(bfs::exists(local))
  {
    mc_rtc::log::info("Loading additional {} configuration: {}", desc, local.string());
    c.load(local.string());
  }
  configs[name] = c;
}

/** Load configurations */
inline void load_configs(const std::string & desc,
                         const std::vector<std::string> & names,
                         const std::vector<std::string> & search_path,
                         const bfs::path & user_path,
                         std::unordered_map<std::string, mc_rtc::Configuration> & configs,
                         const mc_rtc::Configuration & default_config = {},
                         const std::initializer_list<const char *> & filter = {})
{
  for(const auto & name : names) { load_config(desc, name, search_path, user_path, configs, default_config, filter); }
}

} // namespace

void MCGlobalController::GlobalConfiguration::load_controllers_configs()
{
  controllers_configs.clear();
  // Load controller-specific configuration
  load_configs("controller", enabled_controllers, controller_module_paths,
               mc_rtc::user_config_directory_path("controllers"), controllers_configs, config, {"Plugins"});
}

void MCGlobalController::GlobalConfiguration::load_plugin_configs()
{
  load_controller_plugin_configs("", global_plugins);
}

void MCGlobalController::GlobalConfiguration::load_controller_plugin_configs(const std::string & controller,
                                                                             const std::vector<std::string> & plugins)
{
  bfs::path user_config = mc_rtc::user_config_directory_path();
  for(const auto & plugin : plugins)
  {
    auto plugin_c = global_plugin_configs.find(plugin);
    if(plugin_c == global_plugin_configs.end())
    {
      // Global configuration for this plugin has not been loaded yet
      load_config("plugin", plugin, global_plugin_paths, user_config / "plugins", global_plugin_configs);
      plugin_c = global_plugin_configs.find(plugin);
      assert(plugin_c != global_plugin_configs.end());
    }
    if(controller.empty()) { continue; }
    load_config("plugin", plugin, controller_module_paths, user_config / "controllers" / controller / "plugins",
                global_plugin_configs, plugin_c->second, {}, bfs::path("etc") / controller / "plugins");
  }
}

bool MCGlobalController::GlobalConfiguration::enabled(const std::string & ctrl)
{
  return std::find(enabled_controllers.begin(), enabled_controllers.end(), ctrl) != enabled_controllers.end();
}

} // namespace mc_control
