#include <mc_control/mc_global_controller.h>

#include <mc_rbdyn/RobotLoader.h>

#include <mc_rtc/config.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/ros.h>

#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <json/json.h>

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iomanip>

/* Note all service calls except for controller switches are implemented in mc_global_controller_services.cpp */

namespace mc_control
{

MCGlobalController::GlobalConfiguration::GlobalConfiguration(const std::string & conf)
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
  config("RobotModulePaths", robot_module_paths);
  {
    std::string rmp = "";
    config("RobotModulePath", rmp);
    if(rmp.size())
    {
      robot_module_paths.push_back(rmp);
    }
  }
  if(robot_module_paths.size())
  {
    mc_rbdyn::RobotLoader::update_robot_module_path(robot_module_paths);
  }
  std::string robot_name = "HRP2DRC";
  config("MainRobot", robot_name);
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
  config("Default", initial_controller);
  config("Timestep", timestep);
  config("PublishControlState", publish_control_state);
  config("PublishRealState", publish_real_state);
  config("PublishTimestep", publish_timestep);
  config("Log", enable_log);
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

MCGlobalController::MCGlobalController(const std::string & conf)
: config(conf),
  current_ctrl(""), next_ctrl(""),
  controller(0),
  next_controller(0)
{
  controller_loader.reset(new mc_rtc::ObjectLoader<mc_control::MCController>(config.controller_module_paths));
  if(std::find(config.enabled_controllers.begin(), config.enabled_controllers.end(),
            "HalfSitPose") == config.enabled_controllers.end())
  {
    config.enabled_controllers.push_back("HalfSitPose");
  }
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
      LOG_INFO("Create controller " << controller_name)
      if(controller_subname != "")
      {
        controllers[c] = controller_loader->create_object(controller_name, controller_subname, config.main_robot_module, config.timestep, config.config);
      }
      else
      {
        controllers[c] = controller_loader->create_object(c, config.main_robot_module, config.timestep, config.config);
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
  else
  {
    real_robots.load(*config.main_robot_module, config.main_robot_module->rsdf_dir);
    publish_th = std::thread(std::bind(&MCGlobalController::publish_thread, this));
  }
}

MCGlobalController::~MCGlobalController()
{
  publish_th_running = false;
  publish_th.join();
}

void MCGlobalController::init(const std::vector<double> & initq)
{
  init(initq, config.main_robot_module->default_attitude());
}

void MCGlobalController::init(const std::vector<double> & initq, const std::array<double, 7> & initAttitude)
{
  std::vector<std::vector<double>> q = robot().mbc().q;
  q[0] = {std::begin(initAttitude), std::end(initAttitude)};
  const auto & rjo = ref_joint_order();
  for(size_t i = 0; i < rjo.size(); ++i)
  {
    const auto & jn = rjo[i];
    if(robot().hasJoint(jn))
    {
      q[robot().jointIndexByName(jn)][0] = initq[i];
    }
  }
  if(config.main_robot_module->name == "hrp2_drc")
  {
    setGripperCurrentQ({
      {"l_gripper", {initq[31]}},
      {"r_gripper", {initq[23]}}
    });
  }
  else if(config.main_robot_module->name == "hrp4")
  {
    setGripperCurrentQ({
      {"l_gripper", {initq[32], initq[33]}},
      {"r_gripper", {initq[23], initq[24]}}
    });
  }
  controller->reset({q});
  log_header();
}

void MCGlobalController::setSensorPosition(const Eigen::Vector3d & pos)
{
  controller->sensorPos = pos;
}

void MCGlobalController::setSensorOrientation(const Eigen::Vector3d & rpy)
{
  controller->sensorOri = rpy;
}

void MCGlobalController::setSensorLinearVelocity(const Eigen::Vector3d & vel)
{
  controller->sensorLinearVel = vel;
}


void MCGlobalController::setSensorAngularVelocity(const Eigen::Vector3d & vel)
{
  controller->sensorAngularVel = vel;
}

void MCGlobalController::setSensorAcceleration(const Eigen::Vector3d & acc)
{
  controller->sensorAcc = acc;
}

void MCGlobalController::setEncoderValues(const std::vector<double> & eValues)
{
  controller->encoderValues = eValues;
}

void MCGlobalController::setJointTorques(const std::vector<double> & tValues)
{
  controller->jointTorques = tValues;
}

void MCGlobalController::setWrenches(const std::map<std::string, sva::ForceVecd> & wrenches)
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
    log_header();
  }
  if(running)
  {
    bool r = controller->run();
    log_data();
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
    if(controller->grippers.count(gQ.first) == 0)
    {
      LOG_ERROR("Cannot update gripper " << gQ.first)
    }
    else
    {
      controller->grippers[gQ.first]->setCurrentQ(gQ.second);
    }
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

void MCGlobalController::publish_thread()
{
  while(publish_th_running)
  {
    const auto start = std::chrono::high_resolution_clock::now();

    // Publish controlled robot
    if(config.publish_control_state)
    {
      mc_rtc::ROSBridge::update_robot_publisher("control", timestep(), robot(), Eigen::Vector3d::Zero(), controller->getSensorOrientation(), controller->getSensorAngularVelocity(), controller->getSensorAcceleration(), gripperJoints(), gripperQ());
    }

    if(config.publish_real_state)
    {
      const auto& real_q = controller->getEncoderValues();
      if(real_q.size() > 0)
      {
        auto& real_robot = real_robots.robot();
        // Update free flyer
        real_robot.mbc().q[0] = robot().mbc().q[0];
        // Set all joints to encoder values
        int i = 0;
        for(const auto& ref_joint : config.main_robot_module->ref_joint_order())
        {
          const auto joint_index = real_robot.mb().jointIndexByName(ref_joint);
          real_robot.mbc().q[joint_index][0] = real_q[i];
          i++;
        }
        // Publish real robot
        mc_rtc::ROSBridge::update_robot_publisher("real", timestep(), real_robot, Eigen::Vector3d::Zero(), controller->getSensorOrientation(), controller->getSensorAngularVelocity(), controller->getSensorAcceleration(), gripperJoints(), gripperQ());
      }
    }

    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-start).count();
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<long>(1000 * config.publish_timestep) - elapsed));
  }
}

bool MCGlobalController::GoToHalfSitPose()
{
  if(current_ctrl != std::string("HalfSitPose"))
  {
    return EnableController("HalfSitPose");
  }
  return true;
}


void MCGlobalController::log_header()
{
  auto get_log_path = [this]()
  {
    std::stringstream ss;
    auto t = std::time(nullptr);
    auto tm = std::localtime(&t);
    ss << config.log_template
       << "-" << current_ctrl
       << "-" << (1900 + tm->tm_year)
       << "-" << std::setw(2) << std::setfill('0') << (1 + tm->tm_mon)
       << "-" << std::setw(2) << std::setfill('0') << tm->tm_mday
       << "-" << std::setw(2) << std::setfill('0') << tm->tm_hour
       << "-" << std::setw(2) << std::setfill('0') << tm->tm_min
       << "-" << std::setw(2) << std::setfill('0') << tm->tm_sec
       << ".log";
    bfs::path log_path = config.log_directory / bfs::path(ss.str().c_str());
    LOG_INFO("Will log controller outputs to " << log_path)
    return log_path;
  };
  auto log_path = get_log_path();
  if(log_.is_open())
  {
    log_.close();
  }
  log_.open(log_path.string());
  std::stringstream ss_sym;
  ss_sym << config.log_template << "-" << current_ctrl << "-latest.log";
  bfs::path log_sym_path = config.log_directory / bfs::path(ss_sym.str().c_str());
  if(bfs::is_symlink(log_sym_path))
  {
    bfs::remove(log_sym_path);
  }
  if(!bfs::exists(log_sym_path))
  {
    boost::system::error_code ec;
    bfs::create_symlink(log_path, log_sym_path, ec);
    if(!ec)
    {
      LOG_INFO("Updated latest log symlink: " << log_sym_path)
    }
    else
    {
      LOG_INFO("Failed to create latest log symlink: " << ec.message())
    }
  }
  if(log_.is_open())
  {
    log_ << "t";
    for(unsigned int i = 0; i < static_cast<unsigned int>(controller->getEncoderValues().size()); ++i)
    {
      log_ << ";qIn" << i;
    }
    log_ << ";ff_qw;ff_qx;ff_qy;ff_qz;ff_tx;ff_ty;ff_tz";
    for(unsigned int i = 0; i < static_cast<unsigned int>(controller->getEncoderValues().size()); ++i)
    {
      log_ << ";qOut" << i;
    }
    for(unsigned int i = 0; i < static_cast<unsigned int>(controller->getJointTorques().size()); ++i)
    {
      log_ << ";taucIn" << i;
    }
    for(const auto & w : controller->getWrenches())
    {
      const auto& wn = w.first;
      log_ << ";" << wn << "_fx";
      log_ << ";" << wn << "_fy";
      log_ << ";" << wn << "_fz";
      log_ << ";" << wn << "_cx";
      log_ << ";" << wn << "_cy";
      log_ << ";" << wn << "_cz";
    }
    log_ << ";" << "p_x";
    log_ << ";" << "p_y";
    log_ << ";" << "p_z";
    log_ << ";" << "rpy_r";
    log_ << ";" << "rpy_p";
    log_ << ";" << "rpy_y";
    log_ << ";" << "vel_x";
    log_ << ";" << "vel_y";
    log_ << ";" << "vel_z";
    log_ << ";" << "rate_x";
    log_ << ";" << "rate_y";
    log_ << ";" << "rate_z";
    log_ << ";" << "acc_x";
    log_ << ";" << "acc_y";
    log_ << ";" << "acc_z";

    controller->log_header(log_);
    log_ << std::endl;
    log_iter_ = 0;
  }
  else
  {
    LOG_ERROR("Failed to open log file " << log_path)
  }
}

void MCGlobalController::log_data()
{
  if(log_.is_open())
  {
    log_ << log_iter_;
    log_iter_ += timestep();
    for(const auto & qi : controller->getEncoderValues())
    {
      log_ << ";" << qi;
    }
    const auto & ff = controller->robot().mbc().q[0];
    for(const auto & ffi : ff)
    {
      log_ << ";" << ffi;
    }
    const auto & qOut = send(log_iter_).robots_state[0].q;
    for(const auto & jn : ref_joint_order())
    {
      log_ << ";" << qOut.at(jn)[0];
    }
    for(const auto & ti : controller->getJointTorques())
    {
      log_ << ";" << ti;
    }
    for(const auto & w : controller->getWrenches())
    {
      log_ << ";" << w.second.force().x();
      log_ << ";" << w.second.force().y();
      log_ << ";" << w.second.force().z();
      log_ << ";" << w.second.couple().x();
      log_ << ";" << w.second.couple().y();
      log_ << ";" << w.second.couple().z();
    }
    const auto & pIn = controller->getSensorPosition();
    log_ << ";" << pIn.x();
    log_ << ";" << pIn.y();
    log_ << ";" << pIn.z();

    const auto & rpyIn = controller->getSensorOrientation();
    log_ << ";" << rpyIn.x();
    log_ << ";" << rpyIn.y();
    log_ << ";" << rpyIn.z();

    const auto & velIn = controller->getSensorLinearVelocity();
    log_ << ";" << velIn.x();
    log_ << ";" << velIn.y();
    log_ << ";" << velIn.z();

    const auto & rateIn = controller->getSensorAngularVelocity();
    log_ << ";" << rateIn.x();
    log_ << ";" << rateIn.y();
    log_ << ";" << rateIn.z();

    const auto & accIn = controller->getSensorAcceleration();
    log_ << ";" << accIn.x();
    log_ << ";" << accIn.y();
    log_ << ";" << accIn.z();

    controller->log_data(log_);
    log_ << std::endl;
  }
}

}
