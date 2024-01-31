/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/mc_global_controller.h>

#include <mc_control/GlobalPlugin.h>

#ifdef MC_RTC_BUILD_STATIC
#  include <mc_control/ControllerLoader.h>
#  include <mc_control/GlobalPluginLoader.h>
#endif

#include <mc_rbdyn/RobotLoader.h>

#include <mc_rtc/ConfigurationHelpers.h>
#include <mc_rtc/config.h>
#include <mc_rtc/gui/Button.h>
#include <mc_rtc/gui/Form.h>
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/gui/NumberInput.h>
#include <mc_rtc/io_utils.h>
#include <mc_rtc/logging.h>

#include <boost/chrono.hpp>

#include <algorithm>
#include <cstdlib>

namespace mc_control
{

MCGlobalController::PluginHandle::~PluginHandle() {}

MCGlobalController::MCGlobalController(const std::string & conf, std::shared_ptr<mc_rbdyn::RobotModule> rm)
: MCGlobalController(GlobalConfiguration(conf, std::move(rm)))
{
}

MCGlobalController::MCGlobalController(const GlobalConfiguration & conf)
: config(conf), controller_(nullptr), next_controller_(nullptr)
{
  // Display configuration information
  if(conf.enable_gui_server)
  {
    mc_rtc::log::info("GUI server enabled");
    conf.gui_server_configuration.print_serving_information();
  }
  else { mc_rtc::log::info("GUI server disabled"); }
  {
    std::string plugin_str;
    for(const auto & p : conf.global_plugins)
    {
      if(plugin_str.size()) { plugin_str += ", "; }
      plugin_str += p;
      if(std::find(conf.global_plugins_autoload.begin(), conf.global_plugins_autoload.end(), p)
         != conf.global_plugins_autoload.end())
      {
        plugin_str += " (autoload)";
      }
    }
    mc_rtc::log::info("Enabled plugins: {}", plugin_str);
  }
  // Loading plugins
  config.load_plugin_configs();
  try
  {
    plugin_loader_.reset(new mc_rtc::ObjectLoader<mc_control::GlobalPlugin>(
        "MC_RTC_GLOBAL_PLUGIN", config.global_plugin_paths, config.verbose_loader));
  }
  catch(mc_rtc::LoaderException & exc)
  {
    mc_rtc::log::error_and_throw("Failed to initialize plugin loader");
  }
#ifdef MC_RTC_BUILD_STATIC
  GlobalPluginLoader::loader().set_verbosity(config.verbose_loader);
#endif
  for(const auto & plugin : config.global_plugins) { loadPlugin(plugin, "global configuration"); }

  // Loading controller modules
  config.load_controllers_configs();
  try
  {
    controller_loader_.reset(new mc_rtc::ObjectLoader<mc_control::MCController>(
        "MC_RTC_CONTROLLER", config.controller_module_paths, config.verbose_loader));
  }
  catch(mc_rtc::LoaderException & exc)
  {
    mc_rtc::log::error_and_throw("Failed to initialize controller loader");
  }
#ifdef MC_RTC_BUILD_STATIC
  ControllerLoader::loader().set_verbosity(config.verbose_loader);
#endif
  if(std::find(config.enabled_controllers.begin(), config.enabled_controllers.end(), "HalfSitPose")
         == config.enabled_controllers.end()
     && config.include_halfsit_controller)
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
    config.load_controller_plugin_configs(c, config.global_plugins);
    auto ctrl_plugins = mc_rtc::fromVectorOrElement<std::string>(config.controllers_configs[c], "Plugins", {});
    config.load_controller_plugin_configs(c, ctrl_plugins);
  }
  next_ctrl = current_ctrl;
  next_controller_ = nullptr;
  if(current_ctrl == "" || controller_ == nullptr)
  {
    mc_rtc::log::error(
        "No controller selected or selected controller is not enabled, please check your configuration file");
    mc_rtc::log::info("Note common reasons for this error include:\n"
                      "\t- The controller name does not match the name exported by CONTROLLER_CONSTRUCTOR\n"
                      "\t- The controller library is not in a path read by mc_rtc\n"
                      "\t- The controller constuctor segfaults\n"
                      "\t- The controller library hasn't been properly linked");
    mc_rtc::log::error_and_throw("No controller enabled");
  }

  if(config.enable_gui_server)
  {
    server_.reset(new mc_control::ControllerServer(config.timestep, config.gui_server_configuration));
  }
}

MCGlobalController::~MCGlobalController()
{
  // We clear all datastore and gui before (potentially) unloading any libraries
  for(auto & ctl : controllers)
  {
    ctl.second->logger().clear(false);
    ctl.second->gui()->reset();
    ctl.second->datastore().clear();
  }
}

std::shared_ptr<mc_rbdyn::RobotModule> MCGlobalController::get_robot_module()
{
  return config.main_robot_module;
}

std::vector<std::string> MCGlobalController::enabled_controllers() const
{
  std::vector<std::string> ret;
  for(const auto & c : controllers) { ret.push_back(c.first); }
  return ret;
}

std::vector<std::string> MCGlobalController::loaded_controllers() const
{
#ifndef MC_RTC_BUILD_STATIC
  return controller_loader_->objects();
#else
  return ControllerLoader::loader().objects();
#endif
}

std::vector<std::string> MCGlobalController::loaded_robots() const
{
  return mc_rbdyn::RobotLoader::available_robots();
}

std::string MCGlobalController::current_controller() const
{
  return current_ctrl;
}

void MCGlobalController::init(const std::vector<double> & initq, const std::array<double, 7> & initAttitude)
{
  Eigen::Quaterniond q{initAttitude[0], initAttitude[1], initAttitude[2], initAttitude[3]};
  Eigen::Vector3d t{initAttitude[4], initAttitude[5], initAttitude[6]};
  init(initq, sva::PTransformd(q.inverse(), t));
}

void MCGlobalController::init(const std::vector<double> & initq, const sva::PTransformd & initAttitude)
{
  initEncoders(controller().robot(), initq);
  controller().robot().posW(initAttitude);
  for(auto & robot : controller().robots())
  {
    if(robot.robotIndex() == controller().robots().robotIndex()) { continue; }
    initEncoders(robot);
  }
  this->initController();
}

void MCGlobalController::init(const std::vector<double> & initq)
{
  initEncoders(controller().robot(), initq);

  auto & q = controller().robot().mbc().q;
  if(!controller_->config().has("init_pos")
     && !(controller_->config().has(controller().robot().name())
          && controller_->config()(controller().robot().name()).has("init_pos")))
  {
    // Configure initial attitude (requires FK to be computed)
    if(config.init_attitude_from_sensor)
    {
      auto initAttitude = [this](const mc_rbdyn::BodySensor & sensor)
      {
        mc_rtc::log::info("Initializing attitude from body sensor: {}", sensor.name());
        // Update free flyer from body sensor takin into account the kinematics
        // between sensor and body
        const auto & fb = controller().robot().mb().body(0).name();
        sva::PTransformd X_0_s(sensor.orientation(), sensor.position());
        const auto X_s_b = sensor.X_b_s().inv();
        sva::PTransformd X_b_fb = controller().robot().X_b1_b2(sensor.parentBody(), fb);
        sva::PTransformd X_s_fb = X_b_fb * X_s_b;
        auto initPos = X_s_fb * X_0_s;
        controller().robot().posW(initPos);
        controller().realRobot().posW(initPos);
      };
      if(config.init_attitude_sensor.empty()) { initAttitude(controller_->robot().bodySensor()); }
      else
      {
        if(controller_->robot().hasBodySensor(config.init_attitude_sensor))
        {
          initAttitude(controller_->robot().bodySensor(config.init_attitude_sensor));
        }
        else
        {
          mc_rtc::log::error_and_throw<std::invalid_argument>("No body sensor named {}, could not initialize attitude. "
                                                              "Please check your InitAttitudeSensor configuration.",
                                                              config.init_attitude_sensor);
        }
      }
    }
    else
    {
      mc_rtc::log::info("Initializing attitude from robot module: q=[{}]",
                        mc_rtc::io::to_string(controller().robot().module().default_attitude(), ", ", 3));
      if(q[0].size() == 7)
      {
        const auto & initAttitude = controller().robot().module().default_attitude();
        q[0] = {std::begin(initAttitude), std::end(initAttitude)};
        controller().robot().forwardKinematics();
      }
    }
  }
  for(auto & robot : controller().robots())
  {
    if(robot.robotIndex() == controller().robots().robotIndex()) { continue; }
    initEncoders(robot);
  }
  this->initController();
}

void MCGlobalController::init(const std::map<std::string, std::vector<double>> & initqs,
                              const std::map<std::string, sva::PTransformd> & initAttitudes)
{
  init(initqs, initAttitudes, false);
}

void MCGlobalController::init(const std::map<std::string, std::vector<double>> & initqs,
                              const std::map<std::string, sva::PTransformd> & initAttitudes,
                              bool reset)
{
  for(auto & robot : controller().robots())
  {
    auto initq_it = initqs.find(robot.name());
    if(initq_it != initqs.end()) { initEncoders(robot, initq_it->second); }
    else { initEncoders(robot); }
    auto initatt_it = initAttitudes.find(robot.name());
    if(initatt_it != initAttitudes.end()) { robot.posW(initatt_it->second); }
    else
    {
      auto & q = robot.mbc().q;
      if(q[0].size() == 7)
      {
        const auto & initAttitude = robot.module().default_attitude();
        q[0] = {std::begin(initAttitude), std::end(initAttitude)};
        robot.forwardKinematics();
      }
    }
  }
  initController(reset);
}

void MCGlobalController::reset(const std::map<std::string, std::vector<double>> & initqs,
                               const std::map<std::string, sva::PTransformd> & initAttitudes)
{
  controllers.erase(current_ctrl);
  setup_logger_.erase(current_ctrl);
  config.load_controllers_configs();
  AddController(current_ctrl);
  controller_ = controllers[current_ctrl].get();
  init(initqs, initAttitudes, true);
}

void MCGlobalController::initEncoders(mc_rbdyn::Robot & robot, const std::vector<double> & initq)
{
  const auto & rjo = robot.refJointOrder();
  if(initq.size() != rjo.size())
  {
    mc_rtc::log::error_and_throw<std::domain_error>(
        "Could not initialize {} state from encoder sensors: got {} encoder values but {} values are expected\n"
        "Make sure that the MainRobot is compatible with the real/simulated robot used by the interface.",
        robot.name(), initq.size(), rjo.size());
  }

  auto & q = robot.mbc().q;
  for(size_t i = 0; i < rjo.size(); ++i)
  {
    const auto & jn = rjo[i];
    if(robot.hasJoint(jn))
    {
      auto jIndex = robot.jointIndexByName(jn);
      auto jDof = robot.mb().joint(static_cast<int>(jIndex)).dof();
      if(jDof == 1) { q[jIndex][0] = initq[i]; }
      else if(jDof > 1)
      {
        mc_rtc::log::warning("Not using encoder values to initialize {}, please complain to mc_rtc maintainers",
                             rjo[i]);
      }
    }
  }
  for(auto & g : robot.grippers()) { g.get().reset(initq); }
  robot.forwardKinematics();
}

void MCGlobalController::initEncoders(mc_rbdyn::Robot & robot)
{
  // We only need to go through this to initialize the gripper, otherwise the robots are already initialized to the
  // stance configuration
  if(robot.grippers().empty()) { return; }
  const auto & rjo = robot.refJointOrder();
  std::vector<double> rinitq;
  rinitq.reserve(rjo.size());
  for(const auto & j : rjo)
  {
    if(robot.hasJoint(j))
    {
      auto jIndex = robot.jointIndexByName(j);
      const auto & q = robot.mbc().q[jIndex];
      if(q.size())
      {
        for(const auto & qi : q) { rinitq.push_back(qi); }
      }
      else { rinitq.push_back(0.0); }
    }
    else { rinitq.push_back(0.0); }
  }
  for(auto & g : robot.grippers()) { g.get().reset(rinitq); }
}

void MCGlobalController::initController(bool reset)
{
  mc_solver::QPSolver::context_backend(controller_->solver().backend());
  if(config.enable_log) { start_log(); }
  const auto & q = controller().robot().mbc().q;
  controller_->converters_.clear();
  controller_->converters_.reserve(controller_->robots().size());
  for(size_t i = 0; i < controller_->robots().size(); ++i)
  {
    const auto & input = controller_->robots().robot(i);
    auto & output = controller_->outputRobots().robot(i);
    controller_->converters_.emplace_back(input, output, input.module().controlToCanonicalConfig);
  }
  controller_->reset({q});
  controller_->resetObserverPipelines();
  initGUI();
  if(reset)
  {
    for(auto & plugin : plugins_) { plugin.plugin->reset(*this); }
  }
  else
  {
    for(auto & plugin : plugins_) { plugin.plugin->init(*this, config.global_plugin_configs[plugin.name]); }
  }
  resetControllerPlugins();
}

void MCGlobalController::setSensorPosition(const Eigen::Vector3d & pos)
{
  controller().robot().data()->bodySensors[0].position(pos);
}

void MCGlobalController::setSensorPosition(const std::string & robotName, const Eigen::Vector3d & pos)
{
  controller().robot(robotName).data()->bodySensors[0].position(pos);
}

void MCGlobalController::setSensorPositions(const std::map<std::string, Eigen::Vector3d> & poses)
{
  setSensorPositions(controller().robot(), poses);
}

void MCGlobalController::setSensorPositions(const std::string & robotName,
                                            const std::map<std::string, Eigen::Vector3d> & poses)
{
  setSensorPositions(controller().robot(robotName), poses);
}

void MCGlobalController::setSensorPositions(mc_rbdyn::Robot & robot,
                                            const std::map<std::string, Eigen::Vector3d> & poses)
{
  for(const auto & p : poses)
  {
    robot.data()->bodySensors[robot.data()->bodySensorsIndex.at(p.first)].position(p.second);
  }
}

void MCGlobalController::setSensorOrientation(const Eigen::Quaterniond & ori)
{
  controller().robot().data()->bodySensors[0].orientation(ori);
}

void MCGlobalController::setSensorOrientation(const std::string & robotName, const Eigen::Quaterniond & ori)
{
  controller().robot(robotName).data()->bodySensors[0].orientation(ori);
}

void MCGlobalController::setSensorOrientations(const QuaternionMap & oris)
{
  setSensorOrientations(controller().robot(), oris);
}

void MCGlobalController::setSensorOrientations(const std::string & robotName, const QuaternionMap & oris)
{
  setSensorOrientations(controller().robot(robotName), oris);
}

void MCGlobalController::setSensorOrientations(mc_rbdyn::Robot & robot, const QuaternionMap & oris)
{
  for(const auto & o : oris)
  {
    robot.data()->bodySensors[robot.data()->bodySensorsIndex.at(o.first)].orientation(o.second);
  }
}

void MCGlobalController::setSensorLinearVelocity(const Eigen::Vector3d & vel)
{
  controller().robot().data()->bodySensors[0].linearVelocity(vel);
}

void MCGlobalController::setSensorLinearVelocity(const std::string & robotName, const Eigen::Vector3d & vel)
{
  controller().robot(robotName).data()->bodySensors[0].linearVelocity(vel);
}

void MCGlobalController::setSensorLinearVelocities(const std::map<std::string, Eigen::Vector3d> & linearVels)
{
  setSensorLinearVelocities(controller().robot(), linearVels);
}

void MCGlobalController::setSensorLinearVelocities(const std::string & robotName,
                                                   const std::map<std::string, Eigen::Vector3d> & linearVels)
{
  setSensorLinearVelocities(controller().robot(robotName), linearVels);
}

void MCGlobalController::setSensorLinearVelocities(mc_rbdyn::Robot & robot,
                                                   const std::map<std::string, Eigen::Vector3d> & linearVels)
{
  for(const auto & lv : linearVels)
  {
    robot.data()->bodySensors[robot.data()->bodySensorsIndex.at(lv.first)].linearVelocity(lv.second);
  }
}

void MCGlobalController::setSensorAngularVelocity(const Eigen::Vector3d & vel)
{
  controller().robot().data()->bodySensors[0].angularVelocity(vel);
}

void MCGlobalController::setSensorAngularVelocity(const std::string & name, const Eigen::Vector3d & vel)
{
  controller().robot(name).data()->bodySensors[0].angularVelocity(vel);
}

void MCGlobalController::setSensorAngularVelocities(const std::map<std::string, Eigen::Vector3d> & angularVels)
{
  setSensorAngularVelocities(controller().robot(), angularVels);
}

void MCGlobalController::setSensorAngularVelocities(const std::string & name,
                                                    const std::map<std::string, Eigen::Vector3d> & angularVels)
{
  setSensorAngularVelocities(controller().robot(name), angularVels);
}

void MCGlobalController::setSensorAngularVelocities(mc_rbdyn::Robot & robot,
                                                    const std::map<std::string, Eigen::Vector3d> & angularVels)
{
  for(const auto & av : angularVels)
  {
    robot.data()->bodySensors[robot.data()->bodySensorsIndex.at(av.first)].angularVelocity(av.second);
  }
}

void MCGlobalController::setSensorAcceleration(const Eigen::Vector3d & acc)
{
  setSensorLinearAcceleration(acc);
}

void MCGlobalController::setSensorAccelerations(const std::map<std::string, Eigen::Vector3d> & accels)
{
  setSensorLinearAccelerations(accels);
}

void MCGlobalController::setSensorAccelerations(mc_rbdyn::Robot & robot,
                                                const std::map<std::string, Eigen::Vector3d> & accels)
{
  setSensorLinearAccelerations(robot, accels);
}

void MCGlobalController::setSensorLinearAcceleration(const Eigen::Vector3d & acc)
{
  controller().robot().data()->bodySensors[0].linearAcceleration(acc);
}

void MCGlobalController::setSensorLinearAcceleration(const std::string & name, const Eigen::Vector3d & acc)
{
  controller().robot(name).data()->bodySensors[0].linearAcceleration(acc);
}

void MCGlobalController::setSensorLinearAccelerations(const std::map<std::string, Eigen::Vector3d> & accels)
{
  setSensorLinearAccelerations(controller().robot(), accels);
}

void MCGlobalController::setSensorLinearAccelerations(const std::string & name,
                                                      const std::map<std::string, Eigen::Vector3d> & accels)
{
  setSensorLinearAccelerations(controller().robot(name), accels);
}

void MCGlobalController::setSensorLinearAccelerations(mc_rbdyn::Robot & robot,
                                                      const std::map<std::string, Eigen::Vector3d> & accels)
{
  for(const auto & a : accels)
  {
    robot.data()->bodySensors[robot.data()->bodySensorsIndex.at(a.first)].linearAcceleration(a.second);
  }
}

void MCGlobalController::setSensorAngularAcceleration(const Eigen::Vector3d & acc)
{
  controller().robot().data()->bodySensors[0].angularAcceleration(acc);
}

void MCGlobalController::setSensorAngularAccelerations(const std::map<std::string, Eigen::Vector3d> & accels)
{
  setSensorAngularAccelerations(controller().robot(), accels);
}

void MCGlobalController::setSensorAngularAccelerations(const std::string & name,
                                                       const std::map<std::string, Eigen::Vector3d> & accels)
{
  setSensorAngularAccelerations(controller().robot(name), accels);
}

void MCGlobalController::setSensorAngularAccelerations(mc_rbdyn::Robot & robot,
                                                       const std::map<std::string, Eigen::Vector3d> & accels)
{
  for(const auto & a : accels)
  {
    robot.data()->bodySensors[robot.data()->bodySensorsIndex.at(a.first)].angularAcceleration(a.second);
  }
}

void MCGlobalController::setEncoderValues(const std::vector<double> & eValues)
{
  controller().robot().data()->encoderValues = eValues;
}

void MCGlobalController::setEncoderValues(const std::string & robotName, const std::vector<double> & eValues)
{
  controller().robot(robotName).data()->encoderValues = eValues;
}

void MCGlobalController::setEncoderVelocities(const std::vector<double> & eVelocities)
{
  controller().robot().data()->encoderVelocities = eVelocities;
}

void MCGlobalController::setEncoderVelocities(const std::string & robotName, const std::vector<double> & eVelocities)
{
  controller().robot(robotName).data()->encoderVelocities = eVelocities;
}

void MCGlobalController::setJointTorques(const std::vector<double> & tValues)
{
  controller().robot().data()->jointTorques = tValues;
}

void MCGlobalController::setJointTorques(const std::string & robotName, const std::vector<double> & tValues)
{
  controller().robot(robotName).data()->jointTorques = tValues;
}

void MCGlobalController::setWrenches(const std::map<std::string, sva::ForceVecd> & wrenches)
{
  setWrenches(controller_->robot().name(), wrenches);
}

void MCGlobalController::setWrenches(const std::string & robotName,
                                     const std::map<std::string, sva::ForceVecd> & wrenches)
{
  auto & robot = controller().robot(robotName);
  for(const auto & w : wrenches)
  {
    robot.data()->forceSensors[robot.data()->forceSensorsIndex.at(w.first)].wrench(w.second);
  }
}

// deprecated
void MCGlobalController::setWrenches(unsigned int robotIndex, const std::map<std::string, sva::ForceVecd> & wrenches)
{
  auto & robot = controller_->robots().robot(robotIndex);
  setWrenches(robot.name(), wrenches);
}

void MCGlobalController::setJointMotorTemperature(const std::string & joint, double temperature)
{
  setJointMotorTemperature(controller_->robot().name(), joint, temperature);
}

void MCGlobalController::setJointMotorTemperature(const std::string & robotName,
                                                  const std::string & joint,
                                                  double temperature)
{
  auto & robot = controller().robot(robotName);
  auto & sensors = robot.data()->jointSensors;
  sensors[robot.data()->jointJointSensors.at(joint)].motorTemperature(temperature);
}

void MCGlobalController::setJointMotorTemperatures(const std::map<std::string, double> & temperatures)
{
  setJointMotorTemperatures(controller_->robot().name(), temperatures);
}

void MCGlobalController::setJointMotorTemperatures(const std::string & robotName,
                                                   const std::map<std::string, double> & temperatures)
{
  auto & robot = controller().robot(robotName);
  auto & sensors = robot.data()->jointSensors;
  for(const auto & t : temperatures)
  {
    sensors[robot.data()->jointJointSensors.at(t.first)].motorTemperature(t.second);
  }
}

void MCGlobalController::setJointDriverTemperature(const std::string & joint, double temperature)
{
  setJointDriverTemperature(controller_->robot().name(), joint, temperature);
}

void MCGlobalController::setJointDriverTemperature(const std::string & robotName,
                                                   const std::string & joint,
                                                   double temperature)
{
  auto & robot = controller().robot(robotName);
  auto & sensors = robot.data()->jointSensors;
  sensors[robot.data()->jointJointSensors.at(joint)].driverTemperature(temperature);
}

void MCGlobalController::setJointDriverTemperatures(const std::map<std::string, double> & temperatures)
{
  setJointDriverTemperatures(controller_->robot().name(), temperatures);
}

void MCGlobalController::setJointDriverTemperatures(const std::string & robotName,
                                                    const std::map<std::string, double> & temperatures)
{
  auto & robot = controller().robot(robotName);
  auto & sensors = robot.data()->jointSensors;
  for(const auto & t : temperatures)
  {
    sensors[robot.data()->jointJointSensors.at(t.first)].driverTemperature(t.second);
  }
}

void MCGlobalController::setJointMotorCurrent(const std::string & joint, double current)
{
  setJointMotorCurrent(controller_->robot().name(), joint, current);
}

void MCGlobalController::setJointMotorCurrent(const std::string & robotName, const std::string & joint, double current)
{
  auto & robot = controller().robot(robotName);
  auto & sensors = robot.data()->jointSensors;
  sensors[robot.data()->jointJointSensors.at(joint)].motorCurrent(current);
}

void MCGlobalController::setJointMotorCurrents(const std::map<std::string, double> & currents)
{
  setJointMotorCurrents(controller_->robot().name(), currents);
}

void MCGlobalController::setJointMotorCurrents(const std::string & robotName,
                                               const std::map<std::string, double> & currents)
{
  auto & robot = controller().robot(robotName);
  auto & sensors = robot.data()->jointSensors;
  for(const auto & c : currents) { sensors[robot.data()->jointJointSensors.at(c.first)].motorCurrent(c.second); }
}

void MCGlobalController::setJointMotorStatus(const std::string & joint, bool status)
{
  setJointMotorStatus(controller_->robot().name(), joint, status);
}

void MCGlobalController::setJointMotorStatus(const std::string & robotName, const std::string & joint, bool status)
{
  auto & robot = controller().robot(robotName);
  auto & sensors = robot.data()->jointSensors;
  sensors[robot.data()->jointJointSensors.at(joint)].motorStatus(status);
}

void MCGlobalController::setJointMotorStatuses(const std::map<std::string, bool> & statuses)
{
  setJointMotorStatuses(controller_->robot().name(), statuses);
}

void MCGlobalController::setJointMotorStatuses(const std::string & robotName,
                                               const std::map<std::string, bool> & statuses)
{
  auto & robot = controller().robot(robotName);
  auto & sensors = robot.data()->jointSensors;
  for(const auto & s : statuses) { sensors[robot.data()->jointJointSensors.at(s.first)].motorStatus(s.second); }
}

bool MCGlobalController::run()
{
  /** Always pick a steady clock */
  using clock = typename std::conditional<std::chrono::high_resolution_clock::is_steady,
                                          std::chrono::high_resolution_clock, std::chrono::steady_clock>::type;
  /** Helper to converst Tasks' timer */
  auto start_run_t = clock::now();
  /* Check if we need to change the controller this time */
  if(next_controller_)
  {
    mc_rtc::log::info("Switching controllers");
    if(controller_)
    {
      for(auto & bs : next_controller_->robot().data()->bodySensors)
      {
        const auto & current = controller_->robot().bodySensor(bs.name());
        bs.position(current.position());
        bs.orientation(current.orientation());
        bs.linearVelocity(current.linearVelocity());
        bs.angularVelocity(current.angularVelocity());
        bs.linearAcceleration(current.linearAcceleration());
        bs.angularAcceleration(current.angularAcceleration());
      }
      next_controller_->robot().data()->encoderValues = controller_->robot().encoderValues();
      next_controller_->robot().data()->encoderVelocities = controller_->robot().encoderVelocities();
      next_controller_->robot().data()->jointTorques = controller_->robot().jointTorques();
      for(auto & fs : next_controller_->robot().data()->forceSensors)
      {
        fs.wrench(controller_->robot().forceSensor(fs.name()).wrench());
      }
      for(auto & js : next_controller_->robot().data()->jointSensors)
      {
        js.motorTemperature(controller_->robot().jointJointSensor(js.joint()).motorTemperature());
        js.driverTemperature(controller_->robot().jointJointSensor(js.joint()).driverTemperature());
        js.motorCurrent(controller_->robot().jointJointSensor(js.joint()).motorCurrent());
        js.motorStatus(controller_->robot().jointJointSensor(js.joint()).motorStatus());
      }
      next_controller_->realRobot().mbc() = controller_->realRobot().mbc();
    }
    if(!running) { controller_ = next_controller_; }
    else
    {
      // Remove observer pipelines created by MCController::createObserverPipelines
      for(auto & pipeline : controller_->observerPipelines())
      {
        if(controller_->gui_) { pipeline.removeFromGUI(*controller_->gui()); }
        pipeline.removeFromLogger(controller_->logger());
      }
      controller_->stop();
      mc_rtc::log::info("Reset with q[0] = {}", mc_rtc::io::to_string(controller_->robot().mbc().q[0], ", ", 5));
      for(const auto & g : controller_->robot().grippersByName())
      {
        next_controller_->robot().gripper(g.first).reset(*g.second);
      }
      mc_solver::QPSolver::context_backend(next_controller_->solver().backend());
      next_controller_->reset({controller_->robot().mbc().q});
      next_controller_->resetObserverPipelines();
      controller_ = next_controller_;
      /** Reset global plugins */
      for(auto & plugin : plugins_) { plugin.plugin->reset(*this); }
      resetControllerPlugins();
    }
    next_controller_ = nullptr;
    current_ctrl = next_ctrl;
    if(config.enable_log) { start_log(); }
    initGUI();
    mc_rtc::log::success("Controller {} activated", current_ctrl);
  }
  if(running)
  {
    mc_solver::QPSolver::context_backend(controller_->solver().backend());
    for(auto & plugin : plugins_before_)
    {
      auto start_t = clock::now();
      plugin.plugin->before(*this);
      plugin.plugin_before_dt = clock::now() - start_t;
    }
    auto start_observers_run_t = clock::now();
    controller_->runObserverPipelines();
    observers_run_dt = clock::now() - start_observers_run_t;

    auto start_controller_run_t = clock::now();
    bool r = controller_->run();
    auto end_controller_run_t = clock::now();

    for(size_t i = 0; i < controller_->robots().size(); ++i)
    {
      auto & robot = controller_->robots().robot(i);
      auto & realRobot = controller_->realRobots().robot(i);
      auto & outputRobot = controller_->outputRobots().robot(i);
      auto & outputRealRobot = controller_->outputRealRobots().robot(i);
      controller_->converters_[i].convert(robot, outputRobot);
      controller_->converters_[i].convert(realRobot, outputRealRobot);
      const auto & gi = robot.grippers();
      if(!gi.empty())
      {
        for(auto & g : gi) { g.get().run(controller_->timeStep, outputRobot, outputRealRobot); }
        outputRobot.forwardKinematics();
      }
      robot.module().controlToCanonicalPostProcess(robot, outputRobot);
      robot.module().controlToCanonicalPostProcess(realRobot, outputRealRobot);
    }
    if(server_)
    {
      auto start_gui_t = clock::now();
      server_->handle_requests(*controller_->gui_);
      server_->publish(*controller_->gui_);
      gui_dt = clock::now() - start_gui_t;
    }
    controller_run_dt = end_controller_run_t - start_controller_run_t;
    solver_build_and_solve_t = controller_->solver().solveAndBuildTime();
    solver_solve_t = controller_->solver().solveTime();
    if(!r) { running = false; }
    for(auto & plugin : plugins_after_)
    {
      auto start_t = clock::now();
      plugin.plugin->after(*this);
      plugin.plugin_after_dt = clock::now() - start_t;
    }
    if(config.enable_log)
    {
      auto start_log_t = clock::now();
      controller_->logger().log();
      log_dt = clock::now() - start_log_t;
    }
  }
  else
  {
    for(auto & plugin : plugins_before_always_) { plugin->before(*this); }
    controller_run_dt.zero();
    solver_build_and_solve_t = 0;
    solver_solve_t = 0;
    if(server_)
    {
      auto start_gui_t = clock::now();
      server_->handle_requests(*controller_->gui_);
      server_->publish(*controller_->gui_);
      gui_dt = clock::now() - start_gui_t;
    }
    for(auto & plugin : plugins_after_always_) { plugin->after(*this); }
  }
  global_run_dt = clock::now() - start_run_t;
  // Percentage of time not spent inside the user code
  framework_cost = 100 * (1 - controller_run_dt.count() / global_run_dt.count());
  return running;
}

ControllerServer & MCGlobalController::server()
{
  assert(server_);
  return *server_;
}

void MCGlobalController::setGripperTargetQ(const std::string & robot,
                                           const std::string & name,
                                           const std::vector<double> & q)
{
  try
  {
    auto & gripper = controller_->gripper(robot, name);
    gripper.setTargetQ(q);
  }
  catch(const std::exception &)
  {
    mc_rtc::log::error("Cannot set gripper opening for non-existing gripper {} in {}", name, robot);
  }
}

void MCGlobalController::setGripperOpenPercent(const std::string & robot, double pOpen)
{
  auto & r = controller_->robots().robot(robot);
  for(auto & g : r.grippers()) { g.get().setTargetOpening(pOpen); }
}

void MCGlobalController::setGripperOpenPercent(const std::string & robot, const std::string & name, double pOpen)
{
  try
  {
    auto & gripper = controller_->gripper(robot, name);
    gripper.setTargetOpening(pOpen);
  }
  catch(const std::exception &)
  {
    mc_rtc::log::error("Cannot set gripper opening for non-existing gripper {} in {}", name, robot);
  }
}

double MCGlobalController::timestep() const
{
  return config.timestep;
}

const std::vector<std::string> & MCGlobalController::ref_joint_order()
{
  return controller().robot().refJointOrder();
}

bool MCGlobalController::AddController(const std::string & name)
{
  if(controllers.count(name))
  {
    mc_rtc::log::warning("Controller {} already enabled", name);
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
#ifndef MC_RTC_BUILD_STATIC
  auto * controller_loader = controller_loader_.get();
#else
  auto * controller_loader = &ControllerLoader::loader();
#endif
  if(controller_loader->has_object(controller_name))
  {
    mc_rtc::log::info("Create controller {}", controller_name);
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
    controllers[name]->datastore().make_call("Global::EnableController",
                                             [this](const std::string & name) { return EnableController(name); });
    if(config.enable_log)
    {
      controllers[name]->logger().setup(config.log_policy, config.log_directory, config.log_template);
    }
    controllers[name]->createObserverPipelines(config.controllers_configs[name]);
    return true;
  }
  else
  {
    mc_rtc::log::warning("Controller {} enabled in configuration but not available", name);
    return false;
  }
}

const MCGlobalController::GlobalConfiguration & MCGlobalController::configuration() const
{
  return config;
}

void MCGlobalController::add_controller_module_paths(const std::vector<std::string> & paths)
{
  controller_loader_->load_libraries(paths);
}

bool MCGlobalController::AddController(const std::string & name, std::shared_ptr<mc_control::MCController> controller)
{
  if(controllers.count(name) || !controller)
  {
    mc_rtc::log::warning("Controller {} already enabled or invalid pointer passed", name);
    return false;
  }
  controllers[name] = controller;
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
    if(name == current_ctrl) { mc_rtc::log::error("{} controller already enabled.", name); }
    else { mc_rtc::log::error("{} controller not enabled.", name); }
    return false;
  }
}

bool MCGlobalController::GoToHalfSitPose()
{
  if(current_ctrl != std::string("HalfSitPose")) { return EnableController("HalfSitPose"); }
  return true;
}

void MCGlobalController::start_log()
{
  controller_->logger().start(current_ctrl, controller_->timeStep,
                              setup_logger_.find(current_ctrl) != setup_logger_.end());
  setup_log();
  if(server_) { server_->set_logger(controller_->logger_); }
}

void MCGlobalController::refreshLog()
{
  controller_->logger().start(current_ctrl, controller_->timeStep, true);
  setup_log();
}

void MCGlobalController::setup_log()
{
  auto & meta = controller_->logger().meta();
  meta.timestep = controller_->timeStep;
  meta.main_robot = controller_->robot().name();
  meta.main_robot_module = controller_->robot().module().parameters();
  meta.init.clear();
  meta.init_q.clear();
  meta.calibs.clear();
  for(const auto & r : controller_->robots())
  {
    meta.init[r.name()] = r.posW();
    meta.init_q[r.name()] = r.mbc().q;
    for(const auto & fs : r.forceSensors()) { meta.calibs[r.name()][fs.name()] = fs.calib().toConfiguration(); }
  }
  if(setup_logger_.count(current_ctrl)) { return; }
  // Copy controller pointer to avoid lambda issue
  MCController * controller = controller_;
  // Performance measures
  controller->logger().addLogEntry("perf_GlobalRun", [this]() { return global_run_dt.count(); });
  controller->logger().addLogEntry("perf_ControllerRun", [this]() { return controller_run_dt.count(); });
  controller->logger().addLogEntry("perf_ObserversRun", [this]() { return observers_run_dt.count(); });
  controller->logger().addLogEntry("perf_SolverBuildAndSolve", [this]() { return solver_build_and_solve_t; });
  controller->logger().addLogEntry("perf_SolverSolve", [this]() { return solver_solve_t; });
  controller->logger().addLogEntry("perf_Log", [this]() { return log_dt.count(); });
  controller->logger().addLogEntry("perf_Gui", [this]() { return gui_dt.count(); });
  controller->logger().addLogEntry("perf_FrameworkCost", [this]() { return framework_cost; });
  // Log system wall time as nanoseconds since epoch (can be used to manage synchronization with ros)
  controller->logger().addLogEntry("timeWall",
                                   []() -> int64_t
                                   {
                                     int64_t nanoseconds_since_epoch =
                                         std::chrono::system_clock::now().time_since_epoch()
                                         / std::chrono::nanoseconds(1);
                                     return nanoseconds_since_epoch;
                                   });
  setup_logger_[current_ctrl] = true;
}

GlobalPlugin * MCGlobalController::loadPlugin(const std::string & name, const char * requiredBy)
{
#ifdef MC_RTC_BUILD_STATIC
  auto * plugin_loader = &GlobalPluginLoader::loader();
#else
  auto * plugin_loader = plugin_loader_.get();
#endif
  try
  {
    plugins_.emplace_back(name, plugin_loader->create_unique_object(name));
    GlobalPlugin * plugin = plugins_.back().plugin.get();
    const auto & plugin_config = plugins_.back().plugin->configuration();
    if(plugin_config.should_run_before)
    {
      plugins_before_.push_back({plugin, duration_ms{0}});
      if(plugin_config.should_always_run) { plugins_before_always_.push_back(plugin); }
    }
    if(plugin_config.should_run_after)
    {
      plugins_after_.push_back({plugin, duration_ms{0}});
      if(plugin_config.should_always_run) { plugins_after_always_.push_back(plugin); }
    }
    return plugin;
  }
  catch(mc_rtc::LoaderException & exc)
  {
    mc_rtc::log::error(
        "Plugin {} (required by {}) failed to load, functions provided by this plugin will not be available", name,
        requiredBy);
  }
  return nullptr;
}

void MCGlobalController::resetControllerPlugins()
{
  auto next_ctrl_plugins =
      mc_rtc::fromVectorOrElement<std::string>(config.controllers_configs[next_ctrl], "Plugins", {});
  // Remove the plugins that are loaded at the global level
  for(const auto & p : plugins_)
  {
    auto it = std::find(next_ctrl_plugins.begin(), next_ctrl_plugins.end(), p.name);
    if(it != next_ctrl_plugins.end()) { next_ctrl_plugins.erase(it); }
  }
  // Go over controller plugins that are already loaded
  for(auto it = controller_plugins_.begin(); it != controller_plugins_.end();)
  {
    // The plugin is removed if it was loaded by another controller and not needed by this one
    auto next_plugin_it = std::find(next_ctrl_plugins.begin(), next_ctrl_plugins.end(), it->name);
    bool should_remove = (next_plugin_it == next_ctrl_plugins.end());
    if(should_remove)
    {
      // First we remove the plugin from the before/always lists as needed
      auto it_before = std::find_if(plugins_before_.begin(), plugins_before_.end(),
                                    [&](const PluginBefore & p) { return p.plugin == it->plugin.get(); });
      if(it_before != plugins_before_.end()) { plugins_before_.erase(it_before); }
      auto it_before_always = std::find(plugins_before_always_.begin(), plugins_before_always_.end(), it->plugin.get());
      if(it_before_always != plugins_before_always_.end()) { plugins_before_always_.erase(it_before_always); }
      auto it_after = std::find_if(plugins_after_.begin(), plugins_after_.end(),
                                   [&](const PluginAfter & p) { return p.plugin == it->plugin.get(); });
      if(it_after != plugins_after_.end()) { plugins_after_.erase(it_after); }
      auto it_after_always = std::find(plugins_after_always_.begin(), plugins_after_always_.end(), it->plugin.get());
      if(it_after_always != plugins_after_always_.end()) { plugins_after_always_.erase(it_after_always); }
      // Finally we can remove the handle
      it = controller_plugins_.erase(it);
    }
    else
    {
      next_ctrl_plugins.erase(next_plugin_it);
      it->plugin->reset(*this);
      ++it;
    }
  }
  // At this point, next_ctrl_plugins only contains plugins that are required by this controller and not already loaded
  for(const auto & name : next_ctrl_plugins)
  {
    auto plugin = loadPlugin(name, next_ctrl.c_str());
    if(plugin) { plugin->init(*this, config.global_plugin_configs[name]); }
  }
  setup_plugin_log();
}

void MCGlobalController::setup_plugin_log()
{
  auto getPluginName = [this](GlobalPlugin * plugin) -> const std::string &
  {
    for(auto & p : plugins_)
    {
      if(p.plugin.get() == plugin) { return p.name; }
    }
    mc_rtc::log::error_and_throw(
        "Impossible error, searched for a plugin name from a pointer to a plugin that was not loaded");
  };
  for(const auto & plugin : plugins_before_)
  {
    const auto & name = getPluginName(plugin.plugin);
    controller_->logger().addLogEntry(
        fmt::format("perf_Plugins_{}_before", name), [&plugin]() { return plugin.plugin_before_dt.count(); }, true);
  }
  for(const auto & plugin : plugins_after_)
  {
    const auto & name = getPluginName(plugin.plugin);
    controller_->logger().addLogEntry(
        fmt::format("perf_Plugins_{}_after", name), [&plugin]() { return plugin.plugin_after_dt.count(); }, true);
  }
}

} // namespace mc_control
