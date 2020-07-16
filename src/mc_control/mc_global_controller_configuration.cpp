/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/mc_global_controller.h>
#include <mc_observers/ObserverLoader.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rtc/ConfigurationHelpers.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

/* Implementation file for mc_control::MCGlobalController::Configuration */

namespace mc_control
{

MCGlobalController::GlobalConfiguration::GlobalConfiguration(const std::string & conf,
                                                             std::shared_ptr<mc_rbdyn::RobotModule> rm)
{
  // Load default configuration file
  std::string globalPath(mc_rtc::CONF_PATH);
  if(bfs::exists(globalPath))
  {
    mc_rtc::log::info("Loading default global configuration {}", globalPath);
    config.load(globalPath);
  }

#ifndef WIN32
  auto config_path = bfs::path(std::getenv("HOME")) / ".config/mc_rtc/mc_rtc.conf";
#else
  // Should work for Windows Vista and up
  auto config_path = bfs::path(std::getenv("APPDATA")) / "mc_rtc/mc_rtc.conf";
#endif
  // Load user's local configuration if it exists
  if(!bfs::exists(config_path))
  {
    config_path.replace_extension(".yaml");
  }
  if(bfs::exists(config_path))
  {
    mc_rtc::log::info("Loading additional global configuration {}", config_path);
    config.load(config_path.string());
  }
  // Load extra configuration
  if(bfs::exists(conf))
  {
    mc_rtc::log::info("Loading additional global configuration {}", conf);
    config.load(conf);
  }

  ///////////////////////
  //  General options  //
  ///////////////////////
  config("VerboseLoader", verbose_loader);
  config("UseSandbox", use_sandbox);
  config("Timestep", timestep);

  //////////////
  //  Robots  //
  //////////////
  mc_rbdyn::RobotLoader::set_verbosity(verbose_loader);
  config("RobotModulePaths", robot_module_paths);
  mc_rbdyn::RobotLoader::enable_sandboxing(use_sandbox);
  if(config("ClearRobotModulePath", false))
  {
    mc_rbdyn::RobotLoader::clear();
  }
  if(robot_module_paths.size())
  {
    try
    {
      mc_rbdyn::RobotLoader::update_robot_module_path(robot_module_paths);
    }
    catch(const mc_rtc::LoaderException & exc)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("Failed to update robot module path(s)");
    }
  }
  if(rm)
  {
    main_robot_module = rm;
  }
  else
  {
    if(!config.has("MainRobot") || config("MainRobot").size() == 0)
    {
      std::string robot_name = config("MainRobot", std::string{"JVRC1"});
      if(mc_rbdyn::RobotLoader::has_robot(robot_name))
      {
        try
        {
          main_robot_module = mc_rbdyn::RobotLoader::get_robot_module(robot_name);
        }
        catch(const mc_rtc::LoaderException & exc)
        {
          mc_rtc::log::error_and_throw<std::runtime_error>("Failed to create {} to use as a main robot", robot_name);
        }
      }
      else
      {
        mc_rtc::log::error_and_throw<std::runtime_error>(
            "Trying to use {} as main robot but this robot cannot be loaded", robot_name);
      }
    }
    else
    {
      std::vector<std::string> params = config("MainRobot");
      if(mc_rbdyn::RobotLoader::has_robot(params[0]))
      {
        try
        {
          if(params.size() == 1)
          {
            main_robot_module = mc_rbdyn::RobotLoader::get_robot_module(params[0]);
          }
          else if(params.size() == 2)
          {
            main_robot_module = mc_rbdyn::RobotLoader::get_robot_module(params[0], params[1]);
          }
          else if(params.size() == 3)
          {
            main_robot_module = mc_rbdyn::RobotLoader::get_robot_module(params[0], params[1], params[2]);
          }
          else
          {
            throw mc_rtc::LoaderException("Too many parameters given to MainRobot");
          }
        }
        catch(const mc_rtc::LoaderException &)
        {
          mc_rtc::log::error_and_throw<std::runtime_error>("Failed to create main robot using parameters {}",
                                                           config("MainRobot").dump());
        }
      }
      else
      {
        mc_rtc::log::error_and_throw<std::runtime_error>(
            "Trying to use {} as main robot but this robot cannot be loaded", params[0]);
      }
    }
  }
  main_robot_module->expand_stance();
  if(main_robot_module->ref_joint_order().size() == 0)
  {
    main_robot_module->make_default_ref_joint_order();
  }

  /////////////////
  //  Observers  //
  /////////////////
  mc_observers::ObserverLoader::enable_sandboxing(use_sandbox);
  mc_observers::ObserverLoader::set_verbosity(verbose_loader);
  config("ObserverModulePaths", observer_module_paths);
  if(config("ClearObserverModulePath", false))
  {
    mc_observers::ObserverLoader::clear();
  }
  if(!observer_module_paths.empty())
  {
    try
    {
      mc_observers::ObserverLoader::update_module_path(observer_module_paths);
    }
    catch(const mc_rtc::LoaderException & exc)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("Failed to update observer module path(s)");
    }
  }

  // Vector of observers module names (or single observer name)
  enabled_observers = mc_rtc::fromVectorOrElement<std::string>(config, "EnabledObservers", {});
  if(config.has("Observers"))
  {
    const auto & oc = config("Observers");
    for(const auto & observerName : enabled_observers)
    {
      if(oc.has(observerName))
      {
        observer_configs[observerName] = oc(observerName);
      }
      else
      {
        observer_configs[observerName] = {};
      }
    }
  }

  ///////////////
  //  Plugins  //
  ///////////////
  config("GlobalPluginPaths", global_plugin_paths);
  std::string plugin_str = "";
  auto append_plugin = [&plugin_str](const std::string & p, bool autoload) {
    if(plugin_str.size())
    {
      plugin_str += ", ";
    }
    plugin_str += p;
    if(autoload)
    {
      plugin_str += " (autoload)";
    }
  };
  if(!config("ClearGlobalPluginPath", false))
  {
    global_plugin_paths.insert(global_plugin_paths.begin(), mc_rtc::MC_PLUGINS_INSTALL_PREFIX);
    auto autoload_path = bfs::path(mc_rtc::MC_PLUGINS_INSTALL_PREFIX) / "autoload";
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
        global_plugins.push_back(ss.str());
        append_plugin(global_plugins.back(), true);
      }
    }
  }
  std::vector<std::string> plugins = mc_rtc::fromVectorOrElement<std::string>(config, "Plugins", {});
  for(const auto & p : plugins)
  {
    if(std::find(global_plugins.begin(), global_plugins.end(), p) == global_plugins.end())
    {
      global_plugins.push_back(p);
      append_plugin(p, false);
    }
  }
  mc_rtc::log::info("Enabled plugins: {}", plugin_str);

  ///////////////////
  //  Controllers  //
  ///////////////////
  config("ControllerModulePaths", controller_module_paths);
  if(!config("ClearControllerModulePath", false))
  {
    controller_module_paths.insert(controller_module_paths.begin(), mc_rtc::MC_CONTROLLER_INSTALL_PREFIX);
  }
  enabled_controllers = mc_rtc::fromVectorOrElement<std::string>(config, "Enabled", {});
  if(enabled_controllers.size())
  {
    initial_controller = enabled_controllers[0];
  }
  else
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "Enabled entry in mc_rtc must contain at least one controller name");
  }
  config("Default", initial_controller);

  ////////////////////
  // Initialization //
  ////////////////////
  config("InitAttitudeFromSensor", init_attitude_from_sensor);
  config("InitAttitudeSensor", init_attitude_sensor);

  ///////////////
  //  Logging  //
  ///////////////
  config("Log", enable_log);
  config("LogReal", log_real);
  {
    std::string log_policy_str = "non-threaded";
    config("LogPolicy", log_policy_str);
    if(log_policy_str == "threaded")
    {
      log_policy = mc_rtc::Logger::Policy::THREADED;
    }
    else if(log_policy_str == "non-threaded")
    {
      log_policy = mc_rtc::Logger::Policy::NON_THREADED;
    }
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
    if(v.size())
    {
      log_directory = v;
    }
  }
  config("LogTemplate", log_template);

  /////////////////////////
  //  GUI server options //
  /////////////////////////
  if(config.has("GUIServer"))
  {
    auto gui_config = config("GUIServer");
    enable_gui_server = gui_config("Enable", false);
    gui_timestep = gui_config("Timestep", 0.05);
    if(gui_config.has("IPC"))
    {
      auto ipc_config = gui_config("IPC");
      auto socket = ipc_config("Socket", (bfs::temp_directory_path() / "mc_rtc").string());
      gui_server_pub_uris.push_back("ipc://" + socket + "_pub.ipc");
      gui_server_rep_uris.push_back("ipc://" + socket + "_rep.ipc");
    }
    auto handle_section =
        [this, &gui_config](const std::string & section, const std::string & protocol, const std::string & default_host,
                            const std::pair<unsigned int, unsigned int> & default_ports,
                            const std::vector<unsigned int> & used_ports) -> std::vector<unsigned int> {
      if(gui_config.has(section))
      {
        auto prot_config = gui_config(section);
        auto host = prot_config("Host", default_host);
        auto ports = prot_config("Ports", default_ports);
        auto check_port = [&protocol, &used_ports](unsigned int port) {
          if(std::find(used_ports.begin(), used_ports.end(), port) != used_ports.end())
          {
            mc_rtc::log::error(
                "Port {} configured for protocol {} is alread used by another protocol. Expect things to go badly.",
                port, protocol);
          }
        };
        check_port(ports.first);
        check_port(ports.second);
        {
          std::stringstream ss;
          ss << protocol << "://" << host << ":" << ports.first;
          gui_server_pub_uris.push_back(ss.str());
        }
        {
          std::stringstream ss;
          ss << protocol << "://" << host << ":" << ports.second;
          gui_server_rep_uris.push_back(ss.str());
        }
        auto ret = used_ports;
        ret.push_back(ports.first);
        ret.push_back(ports.second);
        return ret;
      }
      return used_ports;
    };
    auto tcp_ports = handle_section("TCP", "tcp", "*", {4242, 4343}, {});
    handle_section("WS", "ws", "*", {8080, 8081}, tcp_ports);
  }
  else
  {
    enable_gui_server = false;
  }
  if(enable_gui_server)
  {
    mc_rtc::log::info("GUI server enabled");
    mc_rtc::log::info("Will serve data on:");
    for(const auto & pub_uri : gui_server_pub_uris)
    {
      mc_rtc::log::info("- {}", pub_uri);
    }
    mc_rtc::log::info("Will handle requests on:");
    for(const auto & rep_uri : gui_server_rep_uris)
    {
      mc_rtc::log::info("- {}", rep_uri);
    }
  }
  else
  {
    mc_rtc::log::info("GUI server disabled");
  }
}

namespace
{

bfs::path conf_or_yaml(bfs::path in)
{
  if(bfs::exists(in))
  {
    return in;
  }
  in.replace_extension(".yaml");
  if(bfs::exists(in))
  {
    return in;
  }
  in.replace_extension(".yml");
  return in;
}

/** Load configurations */
void load_configs(const std::string & desc,
                  const std::vector<std::string> & names,
                  const std::vector<std::string> & search_path,
                  const bfs::path & user_path,
                  std::unordered_map<std::string, mc_rtc::Configuration> & configs,
                  const mc_rtc::Configuration & default_config = {})
{
  for(const auto & name : names)
  {
    mc_rtc::Configuration c;
    c.load(default_config);
    for(const auto & p : search_path)
    {
      bfs::path global = conf_or_yaml(bfs::path(p) / "etc" / (name + ".conf"));
      if(bfs::exists(global))
      {
        mc_rtc::log::info("Loading additional {} configuration: {}", desc, global);
        c.load(global.string());
      }
    }
    bfs::path local = conf_or_yaml(user_path / (name + ".conf"));
    if(bfs::exists(local))
    {
      mc_rtc::log::info("Loading additional {} configuration: {}", desc, local);
      c.load(local.string());
    }
    configs[name] = c;
  }
}

} // namespace

void MCGlobalController::GlobalConfiguration::load_controllers_configs()
{
  // Load controller-specific configuration
  load_configs("controller", enabled_controllers, controller_module_paths,
#ifndef WIN32
               bfs::path(std::getenv("HOME")) / ".config/mc_rtc/controllers",
#else
               bfs::path(std::getenv("APPDATA")) / "mc_rtc/controllers",
#endif
               controllers_configs, config);
}

void MCGlobalController::GlobalConfiguration::load_plugin_configs()
{
  // Load plugins configurations
  load_configs("plugin", global_plugins, global_plugin_paths,
#ifndef WIN32
               bfs::path(std::getenv("HOME")) / ".config/mc_rtc/plugins",
#else
               bfs::path(std::getenv("APPDATA")) / "mc_rtc/plugins",
#endif
               global_plugin_configs);
}

bool MCGlobalController::GlobalConfiguration::enabled(const std::string & ctrl)
{
  return std::find(enabled_controllers.begin(), enabled_controllers.end(), ctrl) != enabled_controllers.end();
}

} // namespace mc_control
