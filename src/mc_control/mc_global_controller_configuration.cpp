/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/mc_global_controller.h>
#include <mc_observers/ObserverLoader.h>
#include <mc_rbdyn/RobotLoader.h>

/* Implementation file for mc_control::MCGlobalController::Configuration */

namespace mc_control
{

MCGlobalController::GlobalConfiguration::GlobalConfiguration(const std::string & conf,
                                                             std::shared_ptr<mc_rbdyn::RobotModule> rm)
: config(mc_rtc::CONF_PATH)
{
#ifndef WIN32
  bfs::path config_path = bfs::path(std::getenv("HOME")) / ".config/mc_rtc/mc_rtc.conf";
#else
  // Should work for Windows Vista and up
  bfs::path config_path = bfs::path(std::getenv("APPDATA")) / "mc_rtc/mc_rtc.conf";
#endif
  if(bfs::exists(config_path))
  {
    LOG_INFO("Loading additional global configuration " << config_path)
    config.load(config_path.string());
  }
  if(bfs::exists(conf))
  {
    LOG_INFO("Loading additional global configuration " << conf)
    config.load(conf);
  }
  config("VerboseLoader", verbose_loader);
  mc_rbdyn::RobotLoader::set_verbosity(verbose_loader);
  config("RobotModulePaths", robot_module_paths);
  config("UseSandbox", use_sandbox);
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
      LOG_ERROR_AND_THROW(std::runtime_error, "Failed to update robot module path(s)")
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
      std::string robot_name = config("MainRobot", std::string{"JVRC-1"});
      if(mc_rbdyn::RobotLoader::has_robot(robot_name))
      {
        try
        {
          main_robot_module = mc_rbdyn::RobotLoader::get_robot_module(robot_name);
        }
        catch(const mc_rtc::LoaderException & exc)
        {
          LOG_ERROR("Failed to create " << robot_name << " to use as a main robot")
          LOG_ERROR_AND_THROW(std::runtime_error, "Failed to create robot")
        }
      }
      else
      {
        LOG_ERROR("Trying to use " << robot_name << " as main robot but this robot cannot be loaded")
        LOG_ERROR_AND_THROW(std::runtime_error, "Main robot not available")
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
          LOG_ERROR("Failed to create main robot using parameters " << config("MainRobot").dump())
          LOG_ERROR_AND_THROW(std::runtime_error, "Failed to create robot")
        }
      }
      else
      {
        LOG_ERROR("Trying to use " << params[0] << " as main robot but this robot cannot be loaded")
        LOG_ERROR_AND_THROW(std::runtime_error, "Main robot not available")
      }
    }
  }
  main_robot_module->expand_stance();
  if(main_robot_module->ref_joint_order().size() == 0)
  {
    main_robot_module->make_default_ref_joint_order();
  }

  ///////////////
  // OBSERVERS
  ///////////////
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
      LOG_ERROR_AND_THROW(std::runtime_error, "Failed to update observer module path(s)")
    }
  }
  config("EnabledObservers", enabled_observers);

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

  config("ControllerModulePaths", controller_module_paths);
  if(!config("ClearControllerModulePath", false))
  {
    controller_module_paths.insert(controller_module_paths.begin(), mc_rtc::MC_CONTROLLER_INSTALL_PREFIX);
  }
  config("Enabled", enabled_controllers);
  if(enabled_controllers.size())
  {
    initial_controller = enabled_controllers[0];
  }
  config("LogReal", log_real);
  config("Default", initial_controller);
  config("Timestep", timestep);
  config("PublishControlState", publish_control_state);
  config("PublishEnvState", publish_env_state);
  config("PublishRealState", publish_real_state);
  config("PublishTimestep", publish_timestep);
  if(publish_timestep < timestep)
  {
    LOG_WARNING("Your ROS publication timestep is lower than your control timestep, your publication timestep will be "
                "effectively set to the control timestep")
  }
  config("Log", enable_log);
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
      LOG_WARNING("Unrecognized LogPolicy entry, will default to non-threaded")
      log_policy = mc_rtc::Logger::Policy::NON_THREADED;
    }
  }
  log_directory = bfs::temp_directory_path();
  {
    std::string v = "";
    config("LogDirectory", v);
    if(v.size())
    {
      log_directory = v;
    }
  }
  config("LogTemplate", log_template);
  /** GUI server options */
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
            LOG_ERROR("Port " << port << " configured for protocol " << protocol
                              << " is alread used by another protocol. Expect things to go badly.")
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
    LOG_INFO("GUI server enabled")
    LOG_INFO("Will serve data on:")
    for(const auto & pub_uri : gui_server_pub_uris)
    {
      LOG_INFO("- " << pub_uri)
    }
    LOG_INFO("Will handle requests on:")
    for(const auto & rep_uri : gui_server_rep_uris)
    {
      LOG_INFO("- " << rep_uri)
    }
  }
  else
  {
    LOG_INFO("GUI server disabled")
  }
  /* Allow the user not to worry about Default if only one controller is enabled */
  if(enabled_controllers.size() == 1)
  {
    initial_controller = enabled_controllers[0];
  }
}

void MCGlobalController::GlobalConfiguration::load_controllers_configs()
{
  // Load controller-specific configuration
  for(const auto & c : enabled_controllers)
  {
    mc_rtc::Configuration conf;
    conf.load(config);
    bfs::path global = bfs::path(mc_rtc::MC_CONTROLLER_INSTALL_PREFIX) / "/etc" / (c + ".conf");
    if(bfs::exists(global))
    {
      LOG_INFO("Loading additional controller configuration" << global)
      conf.load(global.string());
    }
#ifndef WIN32
    bfs::path local = bfs::path(std::getenv("HOME")) / ".config/mc_rtc/controllers" / (c + ".conf");
#else
    bfs::path local = bfs::path(std::getenv("APPDATA")) / "mc_rtc/controllers" / (c + ".conf");
#endif
    if(bfs::exists(local))
    {
      LOG_INFO("Loading additional controller configuration" << local)
      conf.load(local.string());
    }
    controllers_configs[c] = conf;
  }
}

bool MCGlobalController::GlobalConfiguration::enabled(const std::string & ctrl)
{
  return std::find(enabled_controllers.begin(), enabled_controllers.end(), ctrl) != enabled_controllers.end();
}

} // namespace mc_control
