/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/MCController.h>

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/RobotModule.h>
#include <mc_rtc/constants.h>

#include <mc_rtc/ConfigurationHelpers.h>
#include <mc_rtc/config.h>
#include <mc_rtc/gui/Schema.h>
#include <mc_rtc/io_utils.h>
#include <mc_rtc/logging.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <array>
#include <fstream>

namespace mc_control
{

MCController::MCController(std::shared_ptr<mc_rbdyn::RobotModule> robot, double dt) : MCController(robot, dt, {}) {}

MCController::MCController(std::shared_ptr<mc_rbdyn::RobotModule> robot,
                           double dt,
                           const mc_rtc::Configuration & config)
: MCController({robot, mc_rbdyn::RobotLoader::get_robot_module("env/ground")}, dt, config)
{
}

MCController::MCController(const std::vector<std::shared_ptr<mc_rbdyn::RobotModule>> & robots_modules, double dt)
: MCController(robots_modules, dt, {})
{
}

MCController::MCController(const std::vector<std::shared_ptr<mc_rbdyn::RobotModule>> & robots_modules,
                           double dt,
                           const mc_rtc::Configuration & config)
: qpsolver(std::make_shared<mc_solver::QPSolver>(dt)),
  logger_(std::make_shared<mc_rtc::Logger>(mc_rtc::Logger::Policy::NON_THREADED, "", "")),
  gui_(std::make_shared<mc_rtc::gui::StateBuilder>()), config_(config), timeStep(dt)
{
  /* Load robots */
  qpsolver->logger(logger_);
  qpsolver->gui(gui_);
  for(auto rm : robots_modules)
  {
    loadRobot(rm, rm->name);
  }

  if(gui_)
  {
    gui_->addElement({"Global", "Add task"},
                     mc_rtc::gui::Schema("Add MetaTask", "MetaTask", [this](const mc_rtc::Configuration & config) {
                       try
                       {
                         auto t = mc_tasks::MetaTaskLoader::load(this->solver(), config);
                         this->solver().addTask(t);
                       }
                       catch(...)
                       {
                         mc_rtc::log::error("Failed to load MetaTask from request\n{}", config.dump(true));
                       }
                     }));
  }
  /* Initialize constraints and tasks */
  std::array<double, 3> damper = {0.1, 0.01, 0.5};
  contactConstraint = mc_solver::ContactConstraint(timeStep, mc_solver::ContactConstraint::Velocity);
  dynamicsConstraint = mc_solver::DynamicsConstraint(robots(), 0, timeStep, damper, 0.5);
  kinematicsConstraint = mc_solver::KinematicsConstraint(robots(), 0, timeStep, damper, 0.5);
  selfCollisionConstraint = mc_solver::CollisionsConstraint(robots(), 0, 0, timeStep);
  selfCollisionConstraint.addCollisions(solver(), robots_modules[0]->minimalSelfCollisions());
  compoundJointConstraint.reset(new mc_solver::CompoundJointConstraint(robots(), 0, timeStep));
  postureTask = std::make_shared<mc_tasks::PostureTask>(solver(), 0, 10.0, 5.0);
  mc_rtc::log::info("MCController(base) ready");
}

MCController::~MCController()
{
  datastore().clear();
}

mc_rbdyn::Robot & MCController::loadRobot(mc_rbdyn::RobotModulePtr rm, const std::string & name)
{
  loadRobot(rm, name, realRobots(), false);
  return loadRobot(rm, name, robots(), true);
}

mc_rbdyn::Robot & MCController::loadRobot(mc_rbdyn::RobotModulePtr rm,
                                          const std::string & name,
                                          mc_rbdyn::Robots & robots,
                                          bool updateNrVars)
{
  assert(rm);
  auto & r = robots.load(name, *rm);
  r.mbc().gravity = mc_rtc::constants::gravity;
  r.forwardKinematics();
  r.forwardVelocity();
  if(gui_ && updateNrVars)
  {
    auto data = gui_->data();
    if(!data.has("robots"))
    {
      data.array("robots");
    }
    if(!data.has("bodies"))
    {
      data.add("bodies");
    }
    if(!data.has("surfaces"))
    {
      data.add("surfaces");
    }
    if(!data.has("joints"))
    {
      data.add("joints");
    }
    data("robots").push(r.name());
    auto bs = data("bodies").array(r.name());
    for(const auto & b : r.mb().bodies())
    {
      bs.push(b.name());
    }
    data("surfaces").add(r.name(), r.availableSurfaces());
    data("joints").add(r.name(), r.module().ref_joint_order());
    auto name = r.name();
    gui()->addElement({"Robots"}, mc_rtc::gui::Robot(r.name(), [name, this]() -> const mc_rbdyn::Robot & {
                        return this->robot(name);
                      }));
  }
  if(updateNrVars)
  {
    /** Add the robot to the log */
    // If this is the main robot we don't add a prefix
    auto prefix = robots.robot().name() == name ? "" : fmt::format("{}_", name);
    auto entry = [&](const char * entry) { return fmt::format("{}{}", prefix, entry); };
    auto entry_str = [&](const std::string & entry) { return fmt::format("{}{}", prefix, entry); };
    // The robot has some joints so we want to log this state
    if(robot(name).refJointOrder().size())
    {
      logger().addLogEntry(entry("qIn"),
                           [this, name]() -> const std::vector<double> & { return robot(name).encoderValues(); });
      logger().addLogEntry(entry("alphaIn"),
                           [this, name]() -> const std::vector<double> & { return robot(name).encoderVelocities(); });
      logger().addLogEntry(entry("tauIn"),
                           [this, name]() -> const std::vector<double> & { return robot(name).jointTorques(); });
      std::vector<double> qOut(robot(name).refJointOrder().size(), 0);
      logger().addLogEntry(entry("qOut"), [this, name, qOut]() mutable -> const std::vector<double> & {
        auto & robot = this->robot(name);
        for(size_t i = 0; i < qOut.size(); ++i)
        {
          auto mbcIndex = robot.jointIndexInMBC(i);
          if(mbcIndex != -1)
          {
            qOut[i] = robot.mbc().q[static_cast<size_t>(mbcIndex)][0];
          }
        }
        return qOut;
      });
      auto & alphaOut = qOut;
      logger().addLogEntry(entry("alphaOut"), [this, name, alphaOut]() mutable -> const std::vector<double> & {
        auto & robot = this->robot(name);
        for(size_t i = 0; i < alphaOut.size(); ++i)
        {
          auto mbcIndex = robot.jointIndexInMBC(i);
          if(mbcIndex != -1)
          {
            alphaOut[i] = robot.mbc().alpha[static_cast<size_t>(mbcIndex)][0];
          }
        }
        return alphaOut;
      });
      auto & alphaDOut = qOut;
      logger().addLogEntry(entry("alphaDOut"), [this, name, alphaDOut]() mutable -> const std::vector<double> & {
        auto & robot = this->robot(name);
        for(size_t i = 0; i < alphaDOut.size(); ++i)
        {
          auto mbcIndex = robot.jointIndexInMBC(i);
          if(mbcIndex != -1)
          {
            alphaDOut[i] = robot.mbc().alphaD[static_cast<size_t>(mbcIndex)][0];
          }
        }
        return alphaDOut;
      });
      auto & tauOut = qOut;
      logger().addLogEntry(entry("tauOut"), [this, name, tauOut]() mutable -> const std::vector<double> & {
        auto & robot = this->robot(name);
        for(size_t i = 0; i < tauOut.size(); ++i)
        {
          auto mbcIndex = robot.jointIndexInMBC(i);
          if(mbcIndex != -1)
          {
            tauOut[i] = robot.mbc().jointTorque[static_cast<size_t>(mbcIndex)][0];
          }
        }
        return tauOut;
      });
    }
    // Log the floating base position and its estimation if the robot has one
    if(robot(name).mb().joint(0).dof() == 6)
    {
      logger().addLogEntry(entry("ff"),
                           [this, name]() -> const sva::PTransformd & { return robot(name).mbc().bodyPosW[0]; });
      logger().addLogEntry(entry("ff_real"),
                           [this, name]() -> const sva::PTransformd & { return realRobot(name).mbc().bodyPosW[0]; });
    }
    // Log all force sensors
    for(const auto & fs : robot(name).forceSensors())
    {
      const auto & fs_name = fs.name();
      logger().addLogEntry(entry_str(fs.name()), [this, name, fs_name]() -> const sva::ForceVecd & {
        return robot(name).forceSensor(fs_name).wrench();
      });
    }
    // Log all body sensors
    const auto & bodySensors = robot(name).bodySensors();
    for(size_t i = 0; i < bodySensors.size(); ++i)
    {
      const auto & bs_name = bodySensors[i].name();
      logger().addLogEntry(entry_str(name + "_position"), [this, name, bs_name]() -> const Eigen::Vector3d & {
        return robot(name).bodySensor(bs_name).position();
      });
      logger().addLogEntry(entry_str(name + "_orientation"), [this, name, bs_name]() -> const Eigen::Quaterniond & {
        return robot(name).bodySensor(bs_name).orientation();
      });
      logger().addLogEntry(entry_str(name + "_linearVelocity"), [this, name, bs_name]() -> const Eigen::Vector3d & {
        return robot(name).bodySensor(bs_name).linearVelocity();
      });
      logger().addLogEntry(entry_str(name + "_angularVelocity"), [this, name, bs_name]() -> const Eigen::Vector3d & {
        return robot(name).bodySensor(bs_name).angularVelocity();
      });
      logger().addLogEntry(entry_str(name + "_linearAcceleration"), [this, name, bs_name]() -> const Eigen::Vector3d & {
        return robot(name).bodySensor(bs_name).linearAcceleration();
      });
      logger().addLogEntry(entry_str(name + "_angularAcceleration"),
                           [this, name, bs_name]() -> const Eigen::Vector3d & {
                             return robot(name).bodySensor(bs_name).angularAcceleration();
                           });
    }

    /** Now update nr vars */
    solver().updateNrVars();
  }
  return r;
}

void MCController::removeRobot(const std::string & name)
{
  if(robots().hasRobot(name))
  {
    auto prefix = robot().name() == name ? "" : fmt::format("{}_", name);
    auto entry = [&](const char * entry) { return fmt::format("{}{}", prefix, entry); };
    auto entry_str = [&](const std::string & entry) { return fmt::format("{}{}", prefix, entry); };
    const auto & robot = this->robot(name);
    if(robot.refJointOrder().size())
    {
      logger().removeLogEntry(entry("qIn"));
      logger().removeLogEntry(entry("alphaIn"));
      logger().removeLogEntry(entry("tauIn"));
      logger().removeLogEntry(entry("qOut"));
      logger().removeLogEntry(entry("alphaOut"));
      logger().removeLogEntry(entry("alphaDOut"));
      logger().removeLogEntry(entry("tauOut"));
    }
    if(robot.mb().joint(0).dof() == 6)
    {
      logger().removeLogEntry(entry("ff"));
      logger().removeLogEntry(entry("ff_real"));
    }
    for(const auto & fs : robot.forceSensors())
    {
      logger().removeLogEntry(entry_str(fs.name()));
    }
    for(const auto & bs : robot.bodySensors())
    {
      logger().removeLogEntry(entry_str(bs.name() + "_position"));
      logger().removeLogEntry(entry_str(bs.name() + "_orientation"));
      logger().removeLogEntry(entry_str(bs.name() + "_linearVelocity"));
      logger().removeLogEntry(entry_str(bs.name() + "_angularVelocity"));
      logger().removeLogEntry(entry_str(bs.name() + "_linearAcceleration"));
      logger().removeLogEntry(entry_str(bs.name() + "_angularAcceleration"));
    }
  }
  robots().removeRobot(name);
  realRobots().removeRobot(name);
  if(gui_)
  {
    gui_->removeElement({"Robots"}, name);
    auto data = gui_->data();
    std::vector<std::string> robots = data("robots");
    robots.erase(std::find(robots.begin(), robots.end(), name));
    data.add("robots", robots);
    data("bodies").remove(name);
    data("surfaces").remove(name);
  }
  solver().updateNrVars();
}

void MCController::createObserverPipelines(const mc_rtc::Configuration & config)
{
  if(config.has("EnabledObservers") || config.has("RunObservers") || config.has("UpdateObservers"))
  {
    mc_rtc::log::error_and_throw(
        "[{}] The observer pipeline can no longer be configured by \"EnabledObservers\", \"RunObservers\" and "
        "\"UpdateObservers\".\nMultiple "
        "pipelines are now supported, allowing for estimation of multiple robots and/or multiple observations of the "
        "same robot.\nFor details on upgrading, please refer to:\n"
        "- The observer pipelines tutorial: https://jrl-umi3218.github.io/mc_rtc/tutorials/recipes/observers.html\n"
        "- The JSON Schema documentation: https://jrl-umi3218.github.io/mc_rtc/json.html#Observers/ObserverPipelines",
        name_);
  }
  if(!config.has("ObserverPipelines"))
  {
    mc_rtc::log::warning("[MCController::{}] No state observation pipeline configured: the state of the real robots "
                         "will not be estimated",
                         name_);
    return;
  }
  auto pipelineConfigs = mc_rtc::fromVectorOrElement(config, "ObserverPipelines", std::vector<mc_rtc::Configuration>{});
  for(const auto & pipelineConfig : pipelineConfigs)
  {
    observerPipelines_.emplace_back(*this);
    auto & pipeline = observerPipelines_.back();
    pipeline.create(pipelineConfig, timeStep);
    if(pipelineConfig("log", true))
    {
      pipeline.addToLogger(logger());
    }
    if(pipelineConfig("gui", false))
    {
      pipeline.addToGUI(*gui());
    }
  }
}

bool MCController::resetObserverPipelines()
{
  std::string desc;
  for(auto & pipeline : observerPipelines_)
  {
    pipeline.reset();
    if(desc.size())
    {
      desc += "\n";
    }
    desc += "- " + pipeline.desc();
  }
  if(desc.size())
  {
    mc_rtc::log::success("[MCController::{}] State observation pipelines:\n{}", name_, desc);
  }
  return true;
}

bool MCController::runObserverPipelines()
{
  bool success = true;
  for(auto & pipeline : observerPipelines_)
  {
    success = pipeline.run() && success;
  }
  return success;
}

bool MCController::hasObserverPipeline(const std::string & name) const
{
  return std::find_if(observerPipelines_.begin(), observerPipelines_.end(),
                      [&name](const mc_observers::ObserverPipeline & pipeline) { return pipeline.name() == name; })
         != observerPipelines_.end();
}

const std::vector<mc_observers::ObserverPipeline> & MCController::observerPipelines() const
{
  return observerPipelines_;
}

std::vector<mc_observers::ObserverPipeline> & MCController::observerPipelines()
{
  return observerPipelines_;
}

const mc_observers::ObserverPipeline & MCController::observerPipeline(const std::string & name) const
{
  auto pipelineIt =
      std::find_if(observerPipelines_.begin(), observerPipelines_.end(),
                   [&name](const mc_observers::ObserverPipeline & pipeline) { return pipeline.name() == name; });
  if(pipelineIt != observerPipelines_.end())
  {
    return *pipelineIt;
  }
  else
  {
    mc_rtc::log::error_and_throw("Observer pipeline {} does not exist", name);
  }
}

mc_observers::ObserverPipeline & MCController::observerPipeline(const std::string & name)
{
  return const_cast<mc_observers::ObserverPipeline &>(static_cast<const MCController *>(this)->observerPipeline(name));
}

bool MCController::hasObserverPipeline() const
{
  return !observerPipelines_.empty();
}

const mc_observers::ObserverPipeline & MCController::observerPipeline() const
{
  if(!hasObserverPipeline())
  {
    mc_rtc::log::error_and_throw("Controller {} does not have a default observer pipeline", name_);
  }
  return observerPipeline(observerPipelines_.front().name());
}

mc_observers::ObserverPipeline & MCController::observerPipeline()
{
  return const_cast<mc_observers::ObserverPipeline &>(static_cast<const MCController *>(this)->observerPipeline());
}

bool MCController::run()
{
  return run(mc_solver::FeedbackType::None);
}

bool MCController::run(mc_solver::FeedbackType fType)
{
  if(!qpsolver->run(fType))
  {
    mc_rtc::log::error("QP failed to run()");
    return false;
  }
  return true;
}

const mc_solver::QPResultMsg & MCController::send(const double & t)
{
  return qpsolver->send(t);
}

void MCController::reset(const ControllerResetData & reset_data)
{
  std::vector<std::string> supported;
  supported_robots(supported);
  if(supported.size() && std::find(supported.cbegin(), supported.cend(), robot().name()) == supported.end())
  {
    mc_rtc::log::error_and_throw(
        "[MCController] The main robot {} is not supported by this controller. Supported robots are: [{}]",
        robot().name(), mc_rtc::io::to_string(supported));
  }

  robot().mbc().zero(robot().mb());
  robot().mbc().q = reset_data.q;
  postureTask->posture(reset_data.q);
  robot().forwardKinematics();
  robot().forwardVelocity();
}

const mc_rbdyn::Robots & MCController::robots() const
{
  return qpsolver->robots();
}

mc_rbdyn::Robots & MCController::robots()
{
  return qpsolver->robots();
}

const mc_rbdyn::Robot & MCController::robot() const
{
  return qpsolver->robot();
}

mc_rbdyn::Robot & MCController::robot()
{
  return qpsolver->robot();
}

const mc_rbdyn::Robot & MCController::robot(const std::string & name) const
{
  return robots().robot(name);
}

mc_rbdyn::Robot & MCController::robot(const std::string & name)
{
  return robots().robot(name);
}

const mc_rbdyn::Robots & MCController::realRobots() const
{
  return solver().realRobots();
}

mc_rbdyn::Robots & MCController::realRobots()
{
  return solver().realRobots();
}

const mc_rbdyn::Robot & MCController::realRobot() const
{
  return realRobots().robot();
}

mc_rbdyn::Robot & MCController::realRobot()
{
  return realRobots().robot();
}

const mc_rbdyn::Robot & MCController::realRobot(const std::string & name) const
{
  return realRobots().robot(name);
}

mc_rbdyn::Robot & MCController::realRobot(const std::string & name)
{
  return realRobots().robot(name);
}

const mc_rbdyn::Robot & MCController::env() const
{
  return qpsolver->env();
}

mc_rbdyn::Robot & MCController::env()
{
  return qpsolver->env();
}

const mc_solver::QPSolver & MCController::solver() const
{
  assert(qpsolver);
  return *qpsolver;
}

mc_solver::QPSolver & MCController::solver()
{
  assert(qpsolver);
  return *qpsolver;
}

mc_rtc::Logger & MCController::logger()
{
  return *logger_;
}

void MCController::supported_robots(std::vector<std::string> & out) const
{
  out = {};
}

void MCController::stop() {}

Gripper & MCController::gripper(const std::string & robot, const std::string & gripper)
{
  return robots().robot(robot).gripper(gripper);
}

} // namespace mc_control
