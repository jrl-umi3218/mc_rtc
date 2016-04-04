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
    setGripperCurrentQ({
      {"l_gripper", {initq[31]}},
      {"r_gripper", {initq[23]}}
    });
  }
  else if(config.main_robot_module->name == "hrp4")
  {
    auto q_back = controller->robot().mbc().q;
    q.push_back({ 1, 0, 0, 0, 0, 0, 0.76 });
    for (size_t i = 0; i < 6; ++i)
    {
      q.push_back({ initq[i] });
    }
    q.push_back(q_back[7]);
    for (size_t i = 6; i < 12; ++i)
    {
      q.push_back({ initq[i] });
    }
    q.push_back(q_back[14]);
    for (size_t i = 12; i < 25; ++i)
    {
      q.push_back({ initq[i] });
    }
    //R_F
    for (size_t i = 26; i < 34; ++i)
    {
      q.push_back({q_back[i]});
    }
    for (size_t i = 25; i < 34; ++i)
    {
      q.push_back({ initq[i] });
    }
    //L_F
    for (size_t i = 45; i < 53; ++i)
    {
      q.push_back({0.});
    }
    setGripperCurrentQ({
      {"l_gripper", {initq[32], initq[33]}},
      {"r_gripper", {initq[23], initq[24]}}
    });
  }

  for(size_t i = 0; i < q.size(); ++i){
    std::cout << controller->robot().mb().joints()[i].name() << ": ";
    for(size_t j = 0; j < controller->robot().mbc().q[i].size(); ++j){
      if (controller->robot().mbc().q[i].size() == q[i].size()){
        std::cout << "(" << controller->robot().mbc().q[i][j] << " " << q[i][j] << ") ";
      } else {
        std::cout << "ERROR - real size: " << controller->robot().mbc().q[i].size() << " - value:";
        for(size_t k = 0; k < q[i].size(); ++k){
            std::cout << "(" << q[i][k] << ") ";
        }
      }
    }
    std::cout << std::endl;
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

void MCGlobalController::setWrenches(const std::vector<sva::ForceVecd> & wrenches)
{
  controller->setWrenches(wrenches);
}

void MCGlobalController::setActualGripperQ(const std::map<std::string, std::vector<double>> & grippersQ)
{
  for(const auto & gQ : grippersQ)
  {
    assert(controller->grippers.count(gQ.first) > 0);
    controller->grippers[gQ.first]->setActualQ(gQ.second);
  }
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
      for(const auto & g: controller->grippers)
      {
        next_controller->grippers[g.first]->setCurrentQ(g.second->curPosition());
      }
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

std::map<std::string, std::vector<double>> MCGlobalController::gripperQ()
{
  std::map<std::string, std::vector<double>> res;
  for(const auto & g : controller->grippers)
  {
    res[g.first] = g.second->q();
  }
  return res;
}

std::map<std::string, std::vector<std::string>> MCGlobalController::gripperJoints()
{
  std::map<std::string, std::vector<std::string>> res;
  for(const auto & g : controller->grippers)
  {
    res[g.first] = g.second->names;
  }
  return res;
}

std::map<std::string, std::vector<std::string>> MCGlobalController::gripperActiveJoints()
{
  std::map<std::string, std::vector<std::string>> res;
  for(const auto & g : controller->grippers)
  {
    res[g.first] = g.second->active_joints;
  }
  return res;
}

void MCGlobalController::setGripperCurrentQ(const std::map<std::string, std::vector<double>> & gripperQs)
{
  for(const auto & gQ : gripperQs)
  {
    assert(controller->grippers.count(gQ.first) > 0);
    controller->grippers[gQ.first]->setCurrentQ(gQ.second);
  }
}

void MCGlobalController::setGripperTargetQ(const std::string & name, const std::vector<double> & q)
{
  if(controller->grippers.count(name))
  {
    if(controller->grippers[name]->active_idx.size() == q.size())
    {
      controller->grippers[name]->setTargetQ(q);
    }
    else
    {
      LOG_ERROR("Try to set gripper value for " << name << " with the wrong number of values")
    }
  }
  else
  {
    LOG_ERROR("Cannot set gripper value for non-existing gripper " << name)
  }
}

void MCGlobalController::setGripperOpenPercent(double pOpen)
{
  for(const auto & g : controller->grippers)
  {
    setGripperOpenPercent(g.first, pOpen);
  }
}

void MCGlobalController::setGripperOpenPercent(const std::string & name, double pOpen)
{
  if(controller->grippers.count(name))
  {
    controller->grippers[name]->setTargetOpening(pOpen);
  }
  else
  {
    LOG_ERROR("Cannot set gripper opening for non-existing gripper " << name)
  }
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

const std::vector<std::string> & MCGlobalController::ref_joint_order()
{
  return controller->ref_joint_order;
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
