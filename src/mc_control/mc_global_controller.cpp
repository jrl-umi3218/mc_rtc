#include <mc_control/mc_global_controller.h>

#include <mc_rbdyn/RobotLoader.h>

#include <mc_rtc/logging.h>

#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>


#include <json/json.h>
#include <fstream>

/* Note all service calls except for controller switches are implemented in mc_global_controller_services.cpp */

namespace mc_control
{

MCGlobalController::Configuration::Configuration(const std::string & path)
{
  std::ifstream ifs(path);
  if(ifs.bad())
  {
    LOG_ERROR("Failed to open controller configuration file: " << path)
  }
  try
  {
    ifs >> v;
  }
  catch(const std::runtime_error & exc)
  {
    LOG_ERROR("Failed to read configuration file")
    LOG_WARNING(exc.what())
  }

  if(v.isMember("RobotModulePaths"))
  {
    for(const auto & cv : v["RobotModulePaths"])
    {
      robot_module_paths.push_back(cv.asString());
    }
  }
  if(v.isMember("RobotModulePath"))
  {
    robot_module_paths.push_back(v["RobotModulePath"].asString());
  }
  if(robot_module_paths.size())
  {
    mc_rbdyn::RobotLoader::update_robot_module_path(robot_module_paths);
  }
  std::string robot_name = "HRP2DRC";
  if(v.isMember("MainRobot"))
  {
    robot_name = v["MainRobot"].asString();
  }
  if(mc_rbdyn::RobotLoader::has_robot(robot_name))
  {
    main_robot_module = mc_rbdyn::RobotLoader::get_robot_module(robot_name);
  }
  else
  {
    LOG_ERROR("Trying to use " << robot_name << " as main robot but this robot cannot be loaded")
    throw("Main robot not available");
  }

  controller_module_paths.resize(0);
  controller_module_paths.push_back(mc_rtc::MC_CONTROLLER_INSTALL_PREFIX);
  if(v.isMember("ControllerModulePaths"))
  {
    for(const auto & cv : v["ControllerModulePaths"])
    {
      controller_module_paths.push_back(cv.asString());
    }
  }
  if(v.isMember("ControllerModulePath"))
  {
    controller_module_paths.push_back(v["ControllerModulePath"].asString());
  }
  if(v.isMember("Enabled"))
  {
    for(const auto & cv : v["Enabled"])
    {
      enabled_controllers.push_back(cv.asString());
    }
  }
  if(v.isMember("Default"))
  {
    initial_controller = v["Default"].asString();
  }
  else
  {
    if(enabled_controllers.size())
    {
      initial_controller = enabled_controllers[0];
    }
  }
  if(v.isMember("Timestep"))
  {
    timestep = v["Timestep"].asDouble();
  }
  else
  {
    timestep = 0.002;
  }
  /* Allow the user not to worry about Default if only one controller is enabled */
  if(enabled_controllers.size() == 1)
  {
    initial_controller = enabled_controllers[0];
  }
}

bool MCGlobalController::Configuration::enabled(const std::string & ctrl)
{
  return std::find(enabled_controllers.begin(), enabled_controllers.end(), ctrl) != enabled_controllers.end();
}

MCGlobalController::MCGlobalController(const std::string & conf)
: config(conf),
  current_ctrl(""), next_ctrl(""),
  controller(0),
  next_controller(0)
{
  controller_loader.reset(new mc_rtc::ObjectLoader<mc_control::MCController>(config.controller_module_paths));
  for(const auto & c : config.enabled_controllers)
  {
    std::string controller_name = c;
    std::string controller_subname = "";
    size_t sep_pos = c.find('#');
    if(sep_pos != std::string::npos)
    {
      controller_name = c.substr(0, sep_pos);
      controller_subname = c.substr(sep_pos+1);
    }
    if(controller_loader->has_object(controller_name))
    {
      if(controller_subname != "")
      {
        controllers[c] = controller_loader->create_object(controller_name, controller_subname, config.main_robot_module, config.timestep, config.v);
      }
      else
      {
        controllers[c] = controller_loader->create_object(c, config.main_robot_module, config.timestep, config.v);
      }
    }
    else
    {
      LOG_WARNING("Controller " << c << " enabled in configuration but not available");
    }
    if(c == config.initial_controller && controllers.count(c))
    {
      current_ctrl = c;
      controller = controllers[c].get();
    }
  }
  next_ctrl = current_ctrl;
  next_controller = 0;
  if(current_ctrl == "" || controller == 0)
  {
    LOG_ERROR("No controller selected or selected controller is not enabled, please check your configuration file")
    throw("No controller enabled");
  }
}

void MCGlobalController::init(const std::vector<double> & initq)
{
  std::vector<std::vector<double>> q;
  if(config.main_robot_module->name == "hrp2_drc")
  {
    q.push_back({1, 0, 0, 0, 0, 0, 0.76});
    /* The OpenRTM components don't give q in the same order as the QP */
    for(size_t i = 0; i < 24; ++i) // until RARM_LINK7
    {
      q.push_back({initq[i]});
    }
    for(size_t i = 32; i < 37; ++i) // RHAND
    {
      q.push_back({initq[i]});
    }
    for(size_t i = 24; i < 32; ++i) // LARM_LINK*
    {
      q.push_back({initq[i]});
    }
    for(size_t i = 37; i < 42; ++i) // LHAND
    {
      q.push_back({initq[i]});
    }
    setGripperCurrentQ(initq[31], initq[23]);
  }
  else if(config.main_robot_module->name == "hrp4")
  {
    q.push_back({ 1, 0, 0, 0, 0, 0, 0.76 });
    for (size_t i = 0; i < initq.size(); ++i)
    {
      q.push_back({ initq[i] });
    }
    setGripperCurrentQ(initq[44], initq[27]);
  }
  controller->reset({q});
}

void MCGlobalController::setSensorOrientation(const Eigen::Vector3d & rpy)
{
  controller->sensorOri = rpy;
}

void MCGlobalController::setSensorAcceleration(const Eigen::Vector3d & acc)
{
  controller->sensorAcc = acc;
}

void MCGlobalController::setEncoderValues(const std::vector<double> & eValues)
{
  controller->encoderValues = eValues;
}

void MCGlobalController::setWrenches(const std::vector< std::pair<Eigen::Vector3d, Eigen::Vector3d> > & wrenches)
{
  controller->setWrenches(wrenches);
}

void MCGlobalController::setActualGripperQ(double rQ, double lQ)
{
  controller->rgripper->setActualQ(rQ);
  controller->lgripper->setActualQ(lQ);
}

bool MCGlobalController::run()
{
  /* Check if we need to change the controller this time */
  if(next_controller)
  {
    LOG_INFO("Switching controllers")
    if(!running)
    {
      controller = next_controller;
    }
    else
    {
      /*XXX Need to be careful here */
      /*XXX Need to get the current contacts from the controller when needed */
      LOG_INFO("Reset with q[0]")
      std::cout << controller->robot().mbc().q[0][0] << " ";
      std::cout << controller->robot().mbc().q[0][1] << " ";
      std::cout << controller->robot().mbc().q[0][2] << " ";
      std::cout << controller->robot().mbc().q[0][3] << " ";
      std::cout << controller->robot().mbc().q[0][4] << " ";
      std::cout << controller->robot().mbc().q[0][5] << " ";
      std::cout << controller->robot().mbc().q[0][6] << std::endl;
      next_controller->lgripper->setCurrentQ(controller->lgripper->curPosition());
      next_controller->rgripper->setCurrentQ(controller->rgripper->curPosition());
      next_controller->reset({controller->robot().mbc().q});
      controller = next_controller;
    }
    next_controller = 0;
    current_ctrl = next_ctrl;
  }
  if(running)
  {
    bool r = controller->run();
    if(!r) { running = false; }
    return r;
  }
  else
  {
    return false;
  }
}

const QPResultMsg & MCGlobalController::send(const double & t)
{
  return controller->send(t);
}

const mc_rbdyn::Robot & MCGlobalController::robot()
{
  return controller->robot();
}

const std::vector<double> & MCGlobalController::gripperQ(bool lgripper)
{
  if(lgripper)
  {
    return controller->lgripper->q();
  }
  else
  {
    return controller->rgripper->q();
  }
}

void MCGlobalController::setGripperCurrentQ(double lQ, double rQ)
{
  controller->lgripper->setCurrentQ(lQ);
  controller->rgripper->setCurrentQ(rQ);
}

void MCGlobalController::setGripperTargetQ(double lQ, double rQ)
{
  controller->lgripper->setTargetQ(lQ);
  controller->rgripper->setTargetQ(rQ);
}

void MCGlobalController::setLGripperTargetQ(double lQ)
{
  controller->lgripper->setTargetQ(lQ);
}

void MCGlobalController::setRGripperTargetQ(double rQ)
{
  controller->rgripper->setTargetQ(rQ);
}

void MCGlobalController::setGripperOpenPercent(double lQ, double rQ)
{
  controller->lgripper->setTargetOpening(lQ);
  controller->rgripper->setTargetOpening(rQ);
}

std::ostream & MCGlobalController::log_header(std::ostream & os)
{
  return controller->log_header(os);
}

std::ostream & MCGlobalController::log_data(std::ostream & os)
{
  return controller->log_data(os);
}

double MCGlobalController::timestep()
{
  return config.timestep;
}

bool MCGlobalController::EnableController(const std::string & name)
{
  if(name != current_ctrl && controllers.count(name))
  {
    next_ctrl = name;
    next_controller = controllers[name].get();
    return true;
  }
  else
  {
    if(name == current_ctrl)
    {
      LOG_ERROR(name << " controller already enabled.")
    }
    else
    {
      LOG_ERROR(name << " controller not enabled.")
    }
    return false;
  }
}

}
