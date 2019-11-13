/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/mc_global_controller.h>

#include "mc_global_controller_ros_services.h"

#include <mc_rbdyn/RobotLoader.h>

#include <mc_rtc/config.h>
#include <mc_rtc/gui/Button.h>
#include <mc_rtc/gui/Form.h>
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/gui/NumberInput.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/ros.h>

#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <boost/chrono.hpp>

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iomanip>

namespace
{

using gripper_it_t = std::map<std::string, std::shared_ptr<mc_control::Gripper>>::const_iterator;

/** Helper to build the qOut logging */
void addQOutLogEntry(std::function<void(std::vector<double> &)> callback,
                     mc_rtc::Logger & logger,
                     gripper_it_t it,
                     gripper_it_t end,
                     const std::vector<std::string> & refJointOrder)
{
  if(it == end)
  {
    std::vector<double> qOut(refJointOrder.size(), 0);
    auto cb = [callback, qOut]() mutable -> const std::vector<double> & {
      callback(qOut);
      return qOut;
    };
    logger.addLogEntry("qOut", cb);
  }
  else
  {
    auto rjoIndex = [&refJointOrder](const std::string & name) {
      for(size_t j = 0; j < refJointOrder.size(); ++j)
      {
        if(name == refJointOrder[j])
        {
          return static_cast<int>(j);
        }
      }
      return -1;
    };
    const auto & gripper = it->second;
    std::vector<int> gripperToRef;
    for(size_t i = 0; i < gripper->names.size(); ++i)
    {
      gripperToRef.push_back(rjoIndex(gripper->names[i]));
    }
    auto cb = [callback, gripperToRef, gripper](std::vector<double> & qOut) {
      callback(qOut);
      const auto & q = gripper->_q;
      for(size_t i = 0; i < gripperToRef.size(); ++i)
      {
        if(gripperToRef[i] != -1)
        {
          qOut[static_cast<size_t>(gripperToRef[i])] = q[i];
        }
      }
    };
    it++;
    addQOutLogEntry(cb, logger, it, end, refJointOrder);
  }
}

} // namespace

/* Note all service calls except for controller switches are implemented in mc_global_controller_services.cpp */

namespace mc_control
{

MCGlobalController::MCGlobalController(const std::string & conf, std::shared_ptr<mc_rbdyn::RobotModule> rm)
: MCGlobalController(GlobalConfiguration(conf, rm))
{
}

MCGlobalController::MCGlobalController(const GlobalConfiguration & conf)
: config(conf), current_ctrl(""), next_ctrl(""), controller_(nullptr), next_controller_(nullptr),
  real_robots(std::make_shared<mc_rbdyn::Robots>())
{
  // Loading observer modules
  for(const auto & observerName : config.enabled_observers)
  {
    if(mc_observers::ObserverLoader::has_observer(observerName))
    {
      auto observer = mc_observers::ObserverLoader::get_observer(observerName, config.timestep,
                                                                 config.observer_configs[observerName]);

      observers_.push_back(observer);
      observersByName_[observerName] = observer;
    }
    else
    {
      LOG_ERROR("Observer " << observerName << " requested in configuration but not available");
    }
  }

  // Loading controller modules
  config.load_controllers_configs();
  try
  {
    controller_loader.reset(new mc_rtc::ObjectLoader<mc_control::MCController>(
        "MC_RTC_CONTROLLER", config.controller_module_paths, config.use_sandbox, config.verbose_loader));
  }
  catch(mc_rtc::LoaderException & exc)
  {
    LOG_ERROR("Failed to initialize controller loader")
    LOG_ERROR_AND_THROW(std::runtime_error, "Failed to initialize controller loader")
  }
  if(std::find(config.enabled_controllers.begin(), config.enabled_controllers.end(), "HalfSitPose")
     == config.enabled_controllers.end())
  {
    config.enabled_controllers.push_back("HalfSitPose");
  }
  for(const auto & c : config.enabled_controllers)
  {
    AddController(c);
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
    LOG_INFO("Note common reasons for this error include:\n\
             \t- The controller library is not in a path read by mc_rtc\n\
             \t- The controller constuctor segfaults\n\
             \t- The controller library hasn't been properly linked");
    LOG_ERROR_AND_THROW(std::runtime_error, "No controller enabled")
  }
  else
  {
    real_robots->load(*config.main_robot_module);
  }

  mc_rtc::ROSBridge::set_publisher_timestep(config.publish_timestep);
  if(config.enable_log)
  {
    for(auto c : controllers)
    {
      c.second->logger().setup(config.log_policy, config.log_directory, config.log_template);
    }
  }
  if(config.enable_gui_server)
  {
    if(config.gui_server_pub_uris.size() == 0)
    {
      LOG_WARNING("GUI server is enabled but not configured to bind to anything, acting as if it was disabled.")
    }
    else
    {
      server_.reset(new mc_control::ControllerServer(config.timestep, config.gui_timestep, config.gui_server_pub_uris,
                                                     config.gui_server_rep_uris));
    }
  }
  ros_services_.reset(new ROSServicesImpl(mc_rtc::ROSBridge::get_node_handle(), *this));
}

MCGlobalController::~MCGlobalController() {}

std::shared_ptr<mc_rbdyn::RobotModule> MCGlobalController::get_robot_module()
{
  return config.main_robot_module;
}

std::vector<std::string> MCGlobalController::enabled_controllers() const
{
  std::vector<std::string> ret;
  for(const auto & c : controllers)
  {
    ret.push_back(c.first);
  }
  return ret;
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
  if(config.enable_log)
  {
    start_log();
  }
  std::vector<std::vector<double>> q = robot().mbc().q;
  if(q[0].size() == 7)
  {
    q[0] = {std::begin(initAttitude), std::end(initAttitude)};
  }
  const auto & rjo = ref_joint_order();
  for(size_t i = 0; i < rjo.size(); ++i)
  {
    const auto & jn = rjo[i];
    if(robot().hasJoint(jn))
    {
      q[robot().jointIndexByName(jn)][0] = initq[i];
    }
  }
  auto refJointIndex = [&rjo](const std::string & name) {
    for(size_t i = 0; i < rjo.size(); ++i)
    {
      if(name == rjo[i])
      {
        return static_cast<int>(i);
      }
    }
    return -1;
  };
  std::map<std::string, std::vector<double>> gripperInit;
  for(const auto & g_p : controller().grippers)
  {
    const auto & gName = g_p.first;
    const auto & g = *g_p.second;
    gripperInit[gName] = {};
    auto & gQ = gripperInit[gName];
    for(const auto & j : g.active_joints)
    {
      auto jIndex = refJointIndex(j);
      gQ.push_back(jIndex != -1 ? initq[static_cast<size_t>(jIndex)] : 0);
    }
  }
  setGripperCurrentQ(gripperInit);
  controller_->reset({q});
  controller_->resetObservers();
  init_publishers();
  initGUI();
}

void MCGlobalController::setSensorPosition(const Eigen::Vector3d & pos)
{
  robot().bodySensor().position(pos);
  realRobot().bodySensor().position(pos);
}

void MCGlobalController::setSensorPositions(const std::map<std::string, Eigen::Vector3d> & poses)
{
  setSensorPositions(robot(), poses);
  setSensorPositions(realRobot(), poses);
}

void MCGlobalController::setSensorPositions(mc_rbdyn::Robot & robot,
                                            const std::map<std::string, Eigen::Vector3d> & poses)
{
  for(const auto & p : poses)
  {
    robot.bodySensor(p.first).position(p.second);
  }
}

void MCGlobalController::setSensorOrientation(const Eigen::Quaterniond & ori)
{
  robot().bodySensor().orientation(ori);
  realRobot().bodySensor().orientation(ori);
}

void MCGlobalController::setSensorOrientations(const std::map<std::string, Eigen::Quaterniond> & oris)
{
  setSensorOrientations(robot(), oris);
  setSensorOrientations(realRobot(), oris);
}

void MCGlobalController::setSensorOrientations(mc_rbdyn::Robot & robot,
                                               const std::map<std::string, Eigen::Quaterniond> & oris)
{
  for(const auto & o : oris)
  {
    robot.bodySensor(o.first).orientation(o.second);
  }
}

void MCGlobalController::setSensorLinearVelocity(const Eigen::Vector3d & vel)
{
  robot().bodySensor().linearVelocity(vel);
  realRobot().bodySensor().linearVelocity(vel);
}

void MCGlobalController::setSensorLinearVelocities(const std::map<std::string, Eigen::Vector3d> & linearVels)
{
  setSensorLinearVelocities(robot(), linearVels);
  setSensorLinearVelocities(realRobot(), linearVels);
}

void MCGlobalController::setSensorLinearVelocities(mc_rbdyn::Robot & robot,
                                                   const std::map<std::string, Eigen::Vector3d> & linearVels)
{
  for(const auto & lv : linearVels)
  {
    robot.bodySensor(lv.first).linearVelocity(lv.second);
  }
}

void MCGlobalController::setSensorAngularVelocity(const Eigen::Vector3d & vel)
{
  robot().bodySensor().angularVelocity(vel);
  realRobot().bodySensor().angularVelocity(vel);
}

void MCGlobalController::setSensorAngularVelocities(const std::map<std::string, Eigen::Vector3d> & angularVels)
{
  setSensorAngularVelocities(robot(), angularVels);
  setSensorAngularVelocities(realRobot(), angularVels);
}
void MCGlobalController::setSensorAngularVelocities(mc_rbdyn::Robot & robot,
                                                    const std::map<std::string, Eigen::Vector3d> & angularVels)
{
  for(const auto & av : angularVels)
  {
    robot.bodySensor(av.first).angularVelocity(av.second);
  }
}

void MCGlobalController::setSensorAcceleration(const Eigen::Vector3d & acc)
{
  robot().bodySensor().acceleration(acc);
  realRobot().bodySensor().acceleration(acc);
}

void MCGlobalController::setSensorAccelerations(const std::map<std::string, Eigen::Vector3d> & accels)
{
  setSensorAccelerations(robot(), accels);
  setSensorAccelerations(realRobot(), accels);
}

void MCGlobalController::setSensorAccelerations(mc_rbdyn::Robot & robot,
                                                const std::map<std::string, Eigen::Vector3d> & accels)
{
  for(const auto & a : accels)
  {
    robot.bodySensor(a.first).acceleration(a.second);
  }
}

void MCGlobalController::setEncoderValues(const std::vector<double> & eValues)
{
  robot().encoderValues(eValues);
  realRobot().encoderValues(eValues);
}

void MCGlobalController::setEncoderVelocities(const std::vector<double> & eVelocities)
{
  robot().encoderVelocities(eVelocities);
  realRobot().encoderVelocities(eVelocities);
}

void MCGlobalController::setFlexibilityValues(const std::vector<double> & fValues)
{
  robot().flexibilityValues(fValues);
  realRobot().flexibilityValues(fValues);
}

void MCGlobalController::setJointTorques(const std::vector<double> & tValues)
{
  robot().jointTorques(tValues);
  realRobot().jointTorques(tValues);
}

void MCGlobalController::setWrenches(const std::map<std::string, sva::ForceVecd> & wrenches)
{
  setWrenches(controller_->robots().robotIndex(), wrenches);
}

void MCGlobalController::setWrenches(unsigned int robotIndex, const std::map<std::string, sva::ForceVecd> & wrenches)
{
  auto & robot = controller_->robots().robot(robotIndex);
  for(const auto & w : wrenches)
  {
    robot.forceSensor(w.first).wrench(w.second);
    realRobots().robot(robotIndex).forceSensor(w.first).wrench(w.second);
  }
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
  /** Always pick a steady clock */
  using clock = typename std::conditional<std::chrono::high_resolution_clock::is_steady,
                                          std::chrono::high_resolution_clock, std::chrono::steady_clock>::type;
  /** Helper to converst Tasks' timer */
  using boost_ms = boost::chrono::duration<double, boost::milli>;
  using boost_ns = boost::chrono::duration<double, boost::nano>;
  auto start_run_t = clock::now();
  /* Check if we need to change the controller this time */
  if(next_controller_)
  {
    for(auto & observer : observers_)
    {
      observer->removeFromLogger(controller_->logger());
      if(controller_->gui())
      {
        observer->removeFromGUI(*controller_->gui());
      }
    }
    LOG_INFO("Switching controllers")
    if(controller_)
    {
      for(auto & bs : next_controller_->robot().bodySensors())
      {
        const auto & current = controller_->robot().bodySensor(bs.name());
        bs.position(current.position());
        bs.orientation(current.orientation());
        bs.linearVelocity(current.linearVelocity());
        bs.angularVelocity(current.angularVelocity());
        bs.acceleration(current.acceleration());
      }
      next_controller_->robot().encoderValues(controller_->robot().encoderValues());
      next_controller_->robot().jointTorques(controller_->robot().jointTorques());
      for(auto & fs : next_controller_->robot().forceSensors())
      {
        fs.wrench(controller_->robot().forceSensor(fs.name()).wrench());
      }
    }
    if(!running)
    {
      controller_ = next_controller_;
    }
    else
    {
      /*XXX Need to be careful here */
      /*XXX Need to get the current contacts from the controller when needed */
      controller_->stop();
      LOG_INFO("Reset with q[0]")
      std::cout << controller_->robot().mbc().q[0][0] << " ";
      std::cout << controller_->robot().mbc().q[0][1] << " ";
      std::cout << controller_->robot().mbc().q[0][2] << " ";
      std::cout << controller_->robot().mbc().q[0][3] << " ";
      std::cout << controller_->robot().mbc().q[0][4] << " ";
      std::cout << controller_->robot().mbc().q[0][5] << " ";
      std::cout << controller_->robot().mbc().q[0][6] << std::endl;
      for(const auto & g : controller_->grippers)
      {
        next_controller_->grippers[g.first]->setCurrentQ(g.second->curPosition());
      }
      next_controller_->reset({controller_->robot().mbc().q});
      next_controller_->resetObservers();
      controller_ = next_controller_;
      /** Initialize publishers again if the environment changed */
      init_publishers();
    }
    next_controller_ = 0;
    current_ctrl = next_ctrl;
    if(config.enable_log)
    {
      start_log();
    }
    initGUI();
  }
  if(running)
  {
    auto start_controller_run_t = clock::now();
    bool r = controller_->runObservers() && controller_->run();
    auto end_controller_run_t = clock::now();
    if(config.enable_log)
    {
      auto start_log_t = clock::now();
      controller_->logger().log();
      log_dt = clock::now() - start_log_t;
    }
    controller_run_dt = end_controller_run_t - start_controller_run_t;
    solver_build_and_solve_t = boost_ms(boost_ns(controller_->solver().solveAndBuildTime().wall)).count();
    solver_solve_t = boost_ms(boost_ns(controller_->solver().solveTime().wall)).count();
    if(!r)
    {
      running = false;
    }
  }
  else
  {
    controller_run_dt.zero();
    solver_build_and_solve_t = 0;
    solver_solve_t = 0;
  }
  auto start_publish_t = clock::now();
  publish_robots();
  publish_dt = clock::now() - start_publish_t;
  if(server_)
  {
    auto start_gui_t = clock::now();
    server_->handle_requests(*controller_->gui_);
    server_->publish(*controller_->gui_);
    gui_dt = clock::now() - start_gui_t;
  }
  global_run_dt = clock::now() - start_run_t;
  // Percentage of time spent not updating/solving the QP
  framework_cost = 100 * (1 - controller_run_dt.count() / global_run_dt.count());
  return running;
}

const mc_solver::QPResultMsg & MCGlobalController::send(const double & t)
{
  return controller_->send(t);
}

MCController & MCGlobalController::controller()
{
  assert(controller_ != nullptr);
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
    if(controller_->grippers[name]->active_joints.size() == q.size())
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
  return robot().refJointOrder();
}

bool MCGlobalController::AddController(const std::string & name)
{
  if(controllers.count(name))
  {
    LOG_WARNING("Controller " << name << " already enabled")
    return false;
  }
  std::string controller_name = name;
  std::string controller_subname = "";
  size_t sep_pos = name.find('#');
  if(sep_pos != std::string::npos)
  {
    controller_name = name.substr(0, sep_pos);
    controller_subname = name.substr(sep_pos + 1);
  }
  if(controller_loader->has_object(controller_name))
  {
    LOG_INFO("Create controller " << controller_name)
    if(controller_subname != "")
    {
      controllers[name] =
          controller_loader->create_object(controller_name, controller_subname, config.main_robot_module,
                                           config.timestep, config.controllers_configs[name]);
    }
    else
    {
      controllers[name] = controller_loader->create_object(name, config.main_robot_module, config.timestep,
                                                           config.controllers_configs[name]);
    }
    controllers[name]->real_robots = real_robots;
    if(config.enable_log)
    {
      controllers[name]->logger().setup(config.log_policy, config.log_directory, config.log_template);
    }

    // Give each controller access to all observers
    controllers[name]->observers_ = observers_;
    const auto & cc = config.controllers_configs[name];
    const auto runObservers = cc("RunObservers", std::vector<std::string>{});
    const auto updateObservers = cc("UpdateObservers", std::vector<std::string>{});
    // Use controller-specific configuration instead of global configuration
    for(const auto & observerName : runObservers)
    {
      if(observersByName_.count(observerName) > 0)
      {
        auto observer = observersByName_[observerName];
        // If observer is in the "UpdateObserver" configuration, request for
        // update
        if(std::find(updateObservers.begin(), updateObservers.end(), observerName) != updateObservers.end())
        {
          controllers[name]->pipelineObservers_.push_back(std::make_pair(observer, true));
        }
        else
        {
          controllers[name]->pipelineObservers_.push_back(std::make_pair(observer, false));
        }
      }
      else
      {
        LOG_ERROR_AND_THROW(std::runtime_error,
                            "Controller "
                                << controller_name << " requested observer " << observerName
                                << " but this observer is not available. Check your EnabledObservers configuration");
      }
    }
    return true;
  }
  else
  {
    LOG_WARNING("Controller " << name << " enabled in configuration but not available");
    return false;
  }
}

const MCGlobalController::GlobalConfiguration & MCGlobalController::configuration() const
{
  return config;
}

void MCGlobalController::add_controller_module_paths(const std::vector<std::string> & paths)
{
  controller_loader->load_libraries(paths);
}

bool MCGlobalController::AddController(const std::string & name, std::shared_ptr<mc_control::MCController> controller)
{
  if(controllers.count(name) || !controller)
  {
    LOG_WARNING("Controller " << name << " already enabled or invalid pointer passed")
    return false;
  }
  controllers[name] = controller;
  controllers[name]->real_robots = real_robots;
  if(config.enable_log)
  {
    controllers[name]->logger().setup(config.log_policy, config.log_directory, config.log_template);
  }
  return true;
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

void MCGlobalController::init_publishers()
{
  // Publish controlled robot
  if(config.publish_control_state)
  {
    mc_rtc::ROSBridge::init_robot_publisher("control", timestep(), robot());
  }
  // Publish environment state
  if(config.publish_env_state)
  {
    const auto & robots = controller_->robots();
    for(size_t i = 1; i < robots.robots().size(); ++i)
    {
      mc_rtc::ROSBridge::init_robot_publisher("control/env_" + std::to_string(i), timestep(), robots.robot(i));
    }
  }
  // Publish real robot
  if(config.publish_real_state)
  {
    auto & real_robot = real_robots->robot();
    mc_rtc::ROSBridge::init_robot_publisher("real", timestep(), real_robot);
  }
}

void MCGlobalController::publish_robots()
{
  // Publish controlled robot
  if(config.publish_control_state)
  {
    mc_rtc::ROSBridge::update_robot_publisher("control", timestep(), robot(), controller_->grippers);
  }
  // Publish environment state
  if(config.publish_env_state)
  {
    const auto & robots = controller_->robots();
    for(size_t i = 1; i < robots.robots().size(); ++i)
    {
      mc_rtc::ROSBridge::update_robot_publisher("control/env_" + std::to_string(i), timestep(), robots.robot(i));
    }
  }
  // Publish real robot
  if(config.publish_real_state)
  {
    auto & real_robot = real_robots->robot();
    mc_rtc::ROSBridge::update_robot_publisher("real", timestep(), real_robot, controller_->grippers);
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

void MCGlobalController::start_log()
{
  controller_->logger().start(current_ctrl, controller_->timeStep);
  setup_log();
}

void MCGlobalController::refreshLog()
{
  controller_->logger().start(current_ctrl, controller_->timeStep, true);
  setup_log();
}

void MCGlobalController::setup_log()
{
  if(setup_logger_.count(current_ctrl))
  {
    return;
  }
  // Copy controller pointer to avoid lambda issue
  MCController * controller = controller_;
  controller->logger().addLogEntry(
      "qIn", [controller]() -> const std::vector<double> & { return controller->robot().encoderValues(); });
  controller->logger().addLogEntry(
      "ff", [controller]() -> const sva::PTransformd & { return controller->robot().mbc().bodyPosW[0]; });
  // Convert reference order to mbc.q, -1 if the joint does not exist
  std::vector<int> refToQ;
  for(const auto & j : controller->robot().refJointOrder())
  {
    if(controller->robot().hasJoint(j))
    {
      int jIndex = static_cast<int>(controller->robot().jointIndexByName(j));
      if(controller->robot().mb().joint(jIndex).dof() == 1)
      {
        refToQ.push_back(jIndex);
        continue;
      }
    }
    refToQ.push_back(-1);
  }
  auto qOutCb = [controller, refToQ](std::vector<double> & qOut) {
    for(size_t i = 0; i < qOut.size(); ++i)
    {
      if(refToQ[i] != -1)
      {
        qOut[i] = controller->robot().mbc().q[static_cast<size_t>(refToQ[i])][0];
      }
    }
    return qOut;
  };
  addQOutLogEntry(qOutCb, controller->logger(), controller->grippers.cbegin(), controller->grippers.cend(),
                  controller->robot().refJointOrder());
  controller->logger().addLogEntry(
      "tauIn", [controller]() -> const std::vector<double> & { return controller->robot().jointTorques(); });
  for(const auto & fs : controller->robot().forceSensors())
  {
    const auto & fs_name = fs.name();
    controller->logger().addLogEntry(fs.name(), [controller, fs_name]() -> const sva::ForceVecd & {
      return controller->robot().forceSensor(fs_name).wrench();
    });
  }
  controller->logger().addLogEntry(
      "pIn", [controller]() -> const Eigen::Vector3d & { return controller->robot().bodySensor().position(); });
  controller->logger().addLogEntry(
      "rpyIn", [controller]() -> const Eigen::Quaterniond & { return controller->robot().bodySensor().orientation(); });
  controller->logger().addLogEntry(
      "velIn", [controller]() -> const Eigen::Vector3d & { return controller->robot().bodySensor().linearVelocity(); });
  controller->logger().addLogEntry("rateIn", [controller]() -> const Eigen::Vector3d & {
    return controller->robot().bodySensor().angularVelocity();
  });
  controller->logger().addLogEntry(
      "accIn", [controller]() -> const Eigen::Vector3d & { return controller->robot().bodySensor().acceleration(); });

  // Log all other body sensors
  const auto & bodySensors = controller->robot().bodySensors();
  for(size_t i = 1; i < bodySensors.size(); ++i)
  {
    const auto & name = bodySensors[i].name();
    controller->logger().addLogEntry(name + "_pIn", [controller, name]() -> const Eigen::Vector3d & {
      return controller->robot().bodySensor(name).position();
    });
    controller->logger().addLogEntry(name + "_rpyIn", [controller, name]() -> const Eigen::Quaterniond & {
      return controller->robot().bodySensor(name).orientation();
    });
    controller->logger().addLogEntry(name + "_velIn", [controller, name]() -> const Eigen::Vector3d & {
      return controller->robot().bodySensor(name).linearVelocity();
    });
    controller->logger().addLogEntry(name + "_rateIn", [controller, name]() -> const Eigen::Vector3d & {
      return controller->robot().bodySensor(name).angularVelocity();
    });
    controller->logger().addLogEntry(name + "_accIn", [controller, name]() -> const Eigen::Vector3d & {
      return controller->robot().bodySensor(name).acceleration();
    });
  }

  if(config.log_real)
  {
    controller->logger().addLogEntry("realRobot_ff", [controller]() -> const sva::PTransformd & {
      return controller->realRobots().robot().mbc().bodyPosW[0];
    });
    controller->logger().addLogEntry(
        "realRobot_q", [controller]() -> const std::vector<double> & { return controller->robot().encoderValues(); });
    controller->logger().addLogEntry("realRobot_alpha", [controller]() -> const std::vector<double> & {
      return controller->robot().encoderVelocities();
    });
  }

  // Performance measures
  controller->logger().addLogEntry("perf_GlobalRun", [this]() { return global_run_dt.count(); });
  controller->logger().addLogEntry("perf_ControllerRun", [this]() { return controller_run_dt.count(); });
  controller->logger().addLogEntry("perf_SolverBuildAndSolve", [this]() { return solver_build_and_solve_t; });
  controller->logger().addLogEntry("perf_SolverSolve", [this]() { return solver_solve_t; });
  controller->logger().addLogEntry("perf_Log", [this]() { return log_dt.count(); });
  controller->logger().addLogEntry("perf_Publish", [this]() { return publish_dt.count(); });
  controller->logger().addLogEntry("perf_Gui", [this]() { return gui_dt.count(); });
  controller->logger().addLogEntry("perf_FrameworkCost", [this]() { return framework_cost; });
  // Log system wall time as nanoseconds since epoch (can be used to manage synchronization with ros)
  controller->logger().addLogEntry("timeWall", []() -> int64_t {
    int64_t nanoseconds_since_epoch = std::chrono::system_clock::now().time_since_epoch() / std::chrono::nanoseconds(1);
    return nanoseconds_since_epoch;
  });
  setup_logger_[current_ctrl] = true;
}

mc_rbdyn::Robots & MCGlobalController::realRobots()
{
  return *real_robots;
}

mc_rbdyn::Robot & MCGlobalController::realRobot()
{
  return real_robots->robot();
}

} // namespace mc_control
