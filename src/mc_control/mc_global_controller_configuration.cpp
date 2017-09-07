#include <mc_control/mc_global_controller.h>

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
  {
    std::string rmp = "";
    config("RobotModulePath", rmp);
    if(rmp.size())
    {
      robot_module_paths.push_back(rmp);
    }
  }
  config("UseSandbox", use_sandbox);
  mc_rbdyn::RobotLoader::enable_sandboxing(use_sandbox);
  {
    bool clear_rmp = false;
    config("ClearRobotModulePath", clear_rmp);
    if(clear_rmp)
    {
      mc_rbdyn::RobotLoader::clear();
    }
  }
  if(robot_module_paths.size())
  {
    try
    {
      mc_rbdyn::RobotLoader::update_robot_module_path(robot_module_paths);
    }
    catch(const mc_rtc::LoaderException & exc)
    {
      LOG_ERROR("Failed to update robot module path(s)")
      throw std::runtime_error("Failed to update robot module path(s)");
    }
  }
  if(rm)
  {
    main_robot_module = rm;
  }
  else
  {
    std::string robot_name = "HRP2DRC";
    config("MainRobot", robot_name);
    if(mc_rbdyn::RobotLoader::has_robot(robot_name))
    {
      try
      {
        main_robot_module = mc_rbdyn::RobotLoader::get_robot_module(robot_name);
      }
      catch(const mc_rtc::LoaderException & exc)
      {
        LOG_ERROR("Failed to create " << robot_name << " to use as a main robot")
        throw std::runtime_error("Failed to create robot");
      }
    }
    else
    {
      LOG_ERROR("Trying to use " << robot_name << " as main robot but this robot cannot be loaded")
      throw std::runtime_error("Main robot not available");
    }
  }

  controller_module_paths.resize(0);
  bool clear_cmp = false;
  config("ClearControllerModulePath", clear_cmp);
  if(!clear_cmp)
  {
    controller_module_paths.push_back(mc_rtc::MC_CONTROLLER_INSTALL_PREFIX);
  }
  {
    std::vector<std::string> v;
    config("ControllerModulePaths", v);
    for(const auto & cv : v)
    {
      controller_module_paths.push_back(cv);
    }
  }
  {
    std::string v = "";
    config("ControllerModulePaths", v);
    if(v.size())
    {
      controller_module_paths.push_back(v);
    }
  }
  config("Enabled", enabled_controllers);
  if(enabled_controllers.size())
  {
    initial_controller = enabled_controllers[0];
  }
  config("UpdateReal", update_real);
  config("UpdateRealFromSensor", update_real_from_sensors);
  config("Default", initial_controller);
  config("Timestep", timestep);
  config("PublishControlState", publish_control_state);
  config("PublishEnvState", publish_env_state);
  config("PublishRealState", publish_real_state);
  config("PublishTimestep", publish_timestep);
  if(publish_timestep < timestep)
  {
    LOG_WARNING("Your ROS publication timestep is lower than your control timestep, your publication timestep will be effectively set to the control timestep")
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
  /* Allow the user not to worry about Default if only one controller is enabled */
  if(enabled_controllers.size() == 1)
  {
    initial_controller = enabled_controllers[0];
  }
  // Load controller-specific configuration
  for(const auto & c : enabled_controllers)
  {
    bfs::path global = bfs::path(mc_rtc::MC_CONTROLLER_INSTALL_PREFIX) / "/etc" / (c + ".conf");
    if(bfs::exists(global))
    {
      LOG_INFO("Loading additional controller configuration" << global)
      config.load(global.string());
    }
#ifndef WIN32
    bfs::path local = bfs::path(std::getenv("HOME")) / ".config/mc_rtc/controllers" / (c + ".conf");
#else
    bfs::path local = bfs::path(std::getenv("APPDATA")) / "mc_rtc/controllers" / (c + ".conf");
#endif
    if(bfs::exists(local))
    {
      LOG_INFO("Loading additional controller configuration" << local)
      config.load(local.string());
    }
  }
}

bool MCGlobalController::GlobalConfiguration::enabled(const std::string & ctrl)
{
  return std::find(enabled_controllers.begin(), enabled_controllers.end(), ctrl) != enabled_controllers.end();
}

}
