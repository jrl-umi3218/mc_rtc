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

MCGlobalController::MCGlobalController(const std::string & conf)
: config(conf),
  current_ctrl(""), next_ctrl(""),
  controller_(nullptr),
  next_controller_(nullptr)
{
  try
  {
    controller_loader.reset(new mc_rtc::ObjectLoader<mc_control::MCController>(config.controller_module_paths, config.use_sandbox));
  }
  catch(mc_rtc::LoaderException & exc)
  {
    LOG_ERROR("Failed to initialize controller loader")
    throw(std::runtime_error("Failed to initialize controller loader"));
  }
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
      try
      {
        if(controller_subname != "")
        {
          controllers[c] = controller_loader->create_object(controller_name, controller_subname, config.main_robot_module, config.timestep, config.config);
        }
        else
        {
          controllers[c] = controller_loader->create_object(c, config.main_robot_module, config.timestep, config.config);
        }
      }
      catch(const mc_rtc::LoaderException & exc)
      {
        throw std::runtime_error("Failed to create controller");
      }
    }
    else
    {
      LOG_WARNING("Controller " << c << " enabled in configuration but not available");
    }
    if(c == config.initial_controller && controllers.count(c))
    {
      current_ctrl = c;
      controller_ = controllers[c].get();
    }
  }
  next_ctrl = current_ctrl;
  next_controller_ = nullptr;
  if(current_ctrl == "" || controller_ == nullptr)
  {
    LOG_ERROR("No controller selected or selected controller is not enabled, please check your configuration file")
    throw std::runtime_error("No controller enabled");
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

std::vector<std::string> MCGlobalController::loaded_controllers() const
{
  return controller_loader->objects();
}

std::vector<std::string> MCGlobalController::loaded_robots() const
{
  return mc_rbdyn::RobotLoader::available_robots();
}

std::string MCGlobalController::current_controller() const
{
  return current_ctrl;
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
  controller_->reset({q});
  if(config.enable_log)
  {
    logger_.reset(new Logger(config.log_policy, config.log_directory, config.log_template));
    logger_->log_header(current_ctrl, controller_);
  }
}

void MCGlobalController::setSensorPosition(const Eigen::Vector3d & pos)
{
  controller_->sensorPos = pos;
}

void MCGlobalController::setSensorOrientation(const Eigen::Vector3d & rpy)
{
  controller_->sensorOri = rpy;
}

void MCGlobalController::setSensorLinearVelocity(const Eigen::Vector3d & vel)
{
  controller_->sensorLinearVel = vel;
}


void MCGlobalController::setSensorAngularVelocity(const Eigen::Vector3d & vel)
{
  controller_->sensorAngularVel = vel;
}

void MCGlobalController::setSensorAcceleration(const Eigen::Vector3d & acc)
{
  controller_->sensorAcc = acc;
}

void MCGlobalController::setEncoderValues(const std::vector<double> & eValues)
{
  controller_->encoderValues = eValues;
}

void MCGlobalController::setJointTorques(const std::vector<double> & tValues)
{
  controller_->jointTorques = tValues;
}

void MCGlobalController::setWrenches(const std::map<std::string, sva::ForceVecd> & wrenches)
{
  controller_->setWrenches(wrenches);
}

void MCGlobalController::setActualGripperQ(const std::map<std::string, std::vector<double>> & grippersQ)
{
  for(const auto & gQ : grippersQ)
  {
    assert(controller_->grippers.count(gQ.first) > 0);
    controller_->grippers[gQ.first]->setActualQ(gQ.second);
  }
}

bool MCGlobalController::run()
{
  /* Check if we need to change the controller this time */
  if(next_controller_)
  {
    LOG_INFO("Switching controllers")
    if(!running)
    {
      controller_ = next_controller_;
    }
    else
    {
      /*XXX Need to be careful here */
      /*XXX Need to get the current contacts from the controller when needed */
      LOG_INFO("Reset with q[0]")
      std::cout << controller_->robot().mbc().q[0][0] << " ";
      std::cout << controller_->robot().mbc().q[0][1] << " ";
      std::cout << controller_->robot().mbc().q[0][2] << " ";
      std::cout << controller_->robot().mbc().q[0][3] << " ";
      std::cout << controller_->robot().mbc().q[0][4] << " ";
      std::cout << controller_->robot().mbc().q[0][5] << " ";
      std::cout << controller_->robot().mbc().q[0][6] << std::endl;
      for(const auto & g: controller_->grippers)
      {
        next_controller_->grippers[g.first]->setCurrentQ(g.second->curPosition());
      }
      next_controller_->reset({controller_->robot().mbc().q});
      controller_ = next_controller_;
    }
    next_controller_ = 0;
    current_ctrl = next_ctrl;
    if(config.enable_log)
    {
      logger_->log_header(current_ctrl, controller_);
    }
  }
  if(running)
  {
    bool r = controller_->run();
    if(config.enable_log)
    {
      logger_->log_data(*this, controller_);
    }
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
  return controller_->send(t);
}

MCController & MCGlobalController::controller()
{
  assert(controller != nullptr);
  return *controller_;
}

mc_rbdyn::Robot & MCGlobalController::robot()
{
  return controller_->robot();
}

std::map<std::string, std::vector<double>> MCGlobalController::gripperQ()
{
  std::map<std::string, std::vector<double>> res;
  for(const auto & g : controller_->grippers)
  {
    res[g.first] = g.second->q();
  }
  return res;
}

std::map<std::string, std::vector<std::string>> MCGlobalController::gripperJoints()
{
  std::map<std::string, std::vector<std::string>> res;
  for(const auto & g : controller_->grippers)
  {
    res[g.first] = g.second->names;
  }
  return res;
}

std::map<std::string, std::vector<std::string>> MCGlobalController::gripperActiveJoints()
{
  std::map<std::string, std::vector<std::string>> res;
  for(const auto & g : controller_->grippers)
  {
    res[g.first] = g.second->active_joints;
  }
  return res;
}

void MCGlobalController::setGripperCurrentQ(const std::map<std::string, std::vector<double>> & gripperQs)
{
  for(const auto & gQ : gripperQs)
  {
    if(controller_->grippers.count(gQ.first) == 0)
    {
      LOG_ERROR("Cannot update gripper " << gQ.first)
    }
    else
    {
      controller_->grippers[gQ.first]->setCurrentQ(gQ.second);
    }
  }
}

void MCGlobalController::setGripperTargetQ(const std::string & name, const std::vector<double> & q)
{
  if(controller_->grippers.count(name))
  {
    if(controller_->grippers[name]->active_idx.size() == q.size())
    {
      controller_->grippers[name]->setTargetQ(q);
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
  for(const auto & g : controller_->grippers)
  {
    setGripperOpenPercent(g.first, pOpen);
  }
}

void MCGlobalController::setGripperOpenPercent(const std::string & name, double pOpen)
{
  if(controller_->grippers.count(name))
  {
    controller_->grippers[name]->setTargetOpening(pOpen);
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
  return controller_->ref_joint_order;
}

bool MCGlobalController::EnableController(const std::string & name)
{
  if(name != current_ctrl && controllers.count(name))
  {
    next_ctrl = name;
    next_controller_ = controllers[name].get();
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
      mc_rtc::ROSBridge::update_robot_publisher("control", timestep(), robot(), Eigen::Vector3d::Zero(), controller_->getSensorOrientation(), controller_->getSensorAngularVelocity(), controller_->getSensorAcceleration(), gripperJoints(), gripperQ());
    }
    if(config.publish_env_state)
    {
      const auto & robots = controller_->robots();
      for(size_t i = 1; i < robots.robots().size(); ++i)
      {
        std::stringstream ss;
        ss << "control/env_" << i;
        mc_rtc::ROSBridge::update_robot_publisher(ss.str(), timestep(), robots.robot(i), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), {}, {});
      }
    }

    if(config.publish_real_state)
    {
      const auto& real_q = controller_->getEncoderValues();
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
        mc_rtc::ROSBridge::update_robot_publisher("real", timestep(), real_robot, Eigen::Vector3d::Zero(), controller_->getSensorOrientation(), controller_->getSensorAngularVelocity(), controller_->getSensorAcceleration(), gripperJoints(), gripperQ());
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


}
