/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/MCController.h>

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/RobotModule.h>
#include <mc_rtc/constants.h>

#include <mc_rtc/ConfigurationHelpers.h>
#include <mc_rtc/clock.h>
#include <mc_rtc/config.h>
#include <mc_rtc/deprecated.h>
#include <mc_rtc/gui/Schema.h>
#include <mc_rtc/io_utils.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/path.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_solver/ConstraintSetLoader.h>
#include <mc_solver/TVMQPSolver.h>
#include <mc_solver/TasksQPSolver.h>

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#include <array>
#include <fstream>

namespace mc_rtc
{

mc_control::Contact ConfigurationLoader<mc_control::Contact>::load(const mc_rtc::Configuration & config)
{
  return mc_control::Contact(config("r1", std::optional<std::string>{}), config("r2", std::optional<std::string>{}),
                             config("r1Surface"), config("r2Surface"),
                             config("friction", mc_rbdyn::Contact::defaultFriction),
                             config("dof", Eigen::Vector6d::Ones().eval()));
}

} // namespace mc_rtc

namespace mc_control
{

Contact Contact::from_mc_rbdyn(const MCController & ctl, const mc_rbdyn::Contact & contact)
{

  Eigen::Vector6d dof = contact.dof();
  if(ctl.solver().backend() == MCController::Backend::Tasks)
  {
    const auto cId = contact.contactId(ctl.robots());
    if(ctl.contactConstraint->contactConstr()->hasDoFContact(cId))
    {
      dof = ctl.contactConstraint->contactConstr()->dofContact(cId).diagonal();
    }
  }

  return {ctl.robots().robot(contact.r1Index()).name(),
          ctl.robots().robot(contact.r2Index()).name(),
          contact.r1Surface()->name(),
          contact.r2Surface()->name(),
          contact.friction(),
          dof};
}

static thread_local std::string MC_CONTROLLER_LOADING_LOCATION = "";

void MCController::set_loading_location(std::string_view location)
{
  MC_CONTROLLER_LOADING_LOCATION = location;
}

static thread_local std::string MC_CONTROLLER_NAME = "";

void MCController::set_name(std::string_view name)
{
  MC_CONTROLLER_NAME = name;
}

MCController::MCController(std::shared_ptr<mc_rbdyn::RobotModule> robot, double dt, ControllerParameters params)
: MCController(robot, dt, {}, params)
{
}

MCController::MCController(std::shared_ptr<mc_rbdyn::RobotModule> robot,
                           double dt,
                           const mc_rtc::Configuration & config,
                           ControllerParameters params)
: MCController({robot, mc_rbdyn::RobotLoader::get_robot_module("env/ground")}, dt, config, params)
{
}

MCController::MCController(const std::vector<std::shared_ptr<mc_rbdyn::RobotModule>> & robots_modules,
                           double dt,
                           ControllerParameters params)
: MCController(robots_modules, dt, {}, params)
{
}

static inline std::shared_ptr<mc_solver::QPSolver> make_solver(double dt, MCController::Backend backend)
{
  switch(backend)
  {
    case MCController::Backend::Tasks:
      return std::make_shared<mc_solver::TasksQPSolver>(dt);
    case MCController::Backend::TVM:
      return std::make_shared<mc_solver::TVMQPSolver>(dt);
    default:
      mc_rtc::log::error_and_throw("[MCController] Backend {} is not fully supported yet", backend);
  }
}

MCController::MCController(const std::vector<std::shared_ptr<mc_rbdyn::RobotModule>> & robot_modules,
                           double dt,
                           const mc_rtc::Configuration & config,
                           ControllerParameters params)
: qpsolver(make_solver(dt, params.backend_)), outputRobots_(mc_rbdyn::Robots::make()),
  outputRealRobots_(mc_rbdyn::Robots::make()),
  logger_(std::make_shared<mc_rtc::Logger>(mc_rtc::Logger::Policy::NON_THREADED, "", "")),
  gui_(std::make_shared<mc_rtc::gui::StateBuilder>()), config_(config), timeStep(dt), name_(MC_CONTROLLER_NAME),
  loading_location_(MC_CONTROLLER_LOADING_LOCATION)
{
  /* Load robots */
  qpsolver->logger(logger_);
  qpsolver->gui(gui_);
  qpsolver->controller(this);

  std::string main_robot_name = config.find<std::string>("MainRobot", "name").value_or(robot_modules[0]->name);
  loadRobot(robot_modules[0], main_robot_name);
  for(auto it = std::next(robot_modules.cbegin()); it != robot_modules.end(); ++it)
  {
    const auto & rm = *it;
    loadRobot(rm, rm->name);
  }
  /* Load robot-specific configuration depending on parameters */
  auto load_robot_config_into = config;
  if(params.load_robot_config_)
  {
    for(const auto & c : params.load_robot_config_into_)
    {
      if(load_robot_config_into.has(c)) { load_robot_config_into = load_robot_config_into(c); }
      else { load_robot_config_into = load_robot_config_into.add(c); }
    }
  }
  auto load_robot_config = [&](const mc_rbdyn::Robot & robot)
  {
    /** Load the robot specific configuration */
    if(params.load_robot_config_)
    {
      const auto & r_name = params.load_robot_config_with_module_name_ ? robot.module().name : robot.name();
      auto load_into = load_robot_config_into;
      if(!params.overwrite_config_)
      {
        if(load_into.has(r_name)) { load_into = load_into(r_name); }
        else { load_into = load_into.add(r_name); }
      }
      load_into.load(robot_config(r_name));
    }
  };
  for(const auto & r : robots()) { load_robot_config(r); }
  /** Load extra configuration files */
  for(const auto & e : params.extra_configurations_)
  {
    auto load_into = load_robot_config_into;
    if(!params.overwrite_config_)
    {
      if(load_into.has(e)) { load_into = load_into(e); }
      else { load_into = load_into.add(e); }
    }
    load_into.load(robot_config(e));
  }

  if(gui_)
  {
    gui_->addElement({"Global", "Add task"},
                     mc_rtc::gui::Schema("Add MetaTask", "MetaTask",
                                         [this](const mc_rtc::Configuration & config)
                                         {
                                           try
                                           {
                                             auto t = mc_tasks::MetaTaskLoader::load(this->solver(), config);
                                             this->solver().addTask(t);
                                           }
                                           catch(...)
                                           {
                                             mc_rtc::log::error("Failed to load MetaTask from request\n{}",
                                                                config.dump(true));
                                           }
                                         }));
  }
  /* Initialize constraints and tasks */
  contactConstraint.reset(new mc_solver::ContactConstraint(dt));
  std::array<double, 3> damper{0.1, 0.01, 0.5};
  dynamicsConstraint.reset(new mc_solver::DynamicsConstraint(robots(), 0, dt, damper, 0.5));
  kinematicsConstraint.reset(new mc_solver::KinematicsConstraint(robots(), 0, dt, damper, 0.5));
  selfCollisionConstraint.reset(new mc_solver::CollisionsConstraint(robots(), 0, 0, dt));
  selfCollisionConstraint->addCollisions(solver(), robot_modules[0]->minimalSelfCollisions());
  compoundJointConstraint.reset(new mc_solver::CompoundJointConstraint(robots(), 0, timeStep));
  postureTask = std::make_shared<mc_tasks::PostureTask>(solver(), 0, 10.0, 5.0);
  /** Load additional robots from the configuration */
  {
    auto init_robot = [&](const std::string & robotName, const mc_rtc::Configuration & config)
    {
      if(!hasRobot(robotName)) return;
      auto & robot = robots().robot(robotName);
      auto & realRobot = realRobots().robot(robotName);
      /** Set initial robot base pose */
      if(config.has("init_pos"))
      {
        robot.posW(config("init_pos"));
        realRobot.posW(robot.posW());
      }
      /** Load additional frames */
      {
        auto frames = config("frames", std::vector<mc_rbdyn::RobotModule::FrameDescription>{});
        robot.makeFrames(frames);
        realRobot.makeFrames(frames);
      }
    };
    if(config.has("init_pos"))
    {
      mc_rtc::log::deprecated("mc_control::fsm::Controller", "init_pos",
                              fmt::format("robots/{}/init_pos", robot().name()));
      init_robot(robot().name(), config);
    }
    auto config_robots = config("robots", mc_rtc::Configuration{});
    auto config_robots_keys = config_robots.keys();
    for(const auto & rname : config_robots_keys)
    {
      auto rconfig = config("robots")(rname);
      if(rname == robot().name())
      {
        if(config.has("init_pos") && rconfig.has("init_pos"))
        {
          mc_rtc::log::error_and_throw("You have both a global \"init_pos\" entry and a \"robots/{}/init_pos\" entry "
                                       "in your FSM configuration. Please use \"robots/{}/init_pos\" only.",
                                       robot().name(), robot().name());
        }
        init_robot(rname, rconfig);
      }
      else if(!rconfig.has("module"))
      {
        // This is not the main robot but the configuration does not have a
        // robot module specified. This can happen if the user intended his
        // controller to run with multiple main robots each requiring a
        // different configuration. In this case, the configuration is simply
        // ignored as it does not correspond to any robot currently loaded
        // within this controller.
      }
      else
      {
        auto params = [&]() -> std::vector<std::string>
        {
          auto module = rconfig("module");
          if(module.isArray()) { return module.operator std::vector<std::string>(); }
          std::vector<std::string> params = rconfig("params", std::vector<std::string>{});
          params.insert(params.begin(), module.operator std::string());
          return params;
        }();
        auto rm = mc_rbdyn::RobotLoader::get_robot_module(params);
        if(!rm) { mc_rtc::log::error_and_throw("Failed to load {} as specified in configuration", rname); }
        auto & robot = loadRobot(rm, rname);
        load_robot_config(robot);
        rconfig = config("robots")(rname);
        init_robot(rname, rconfig);
      }
    }
    mc_rtc::log::info("Robots loaded in controller:");
    for(const auto & r : robots()) { mc_rtc::log::info("- {}", r.name()); }
  }
  /** Load global constraints (robots' kinematics/dynamics constraints and contact constraint */
  {
    auto config_constraints = config("constraints", std::vector<mc_rtc::Configuration>{});
    for(const auto & cc : config_constraints)
    {
      constraints_.emplace_back(mc_solver::ConstraintSetLoader::load(solver(), cc));
      if(static_cast<std::string>(cc("type")) == "contact")
      {
        contact_constraint_ = std::static_pointer_cast<mc_solver::ContactConstraint>(constraints_.back());
      }
      /*FIXME Add a name mechanism in ConstraintSet to get information here */
      solver().addConstraintSet(*constraints_.back());
    }
    if(!contact_constraint_)
    {
      contact_constraint_ =
          std::shared_ptr<mc_solver::ContactConstraint>(contactConstraint.get(), [](mc_solver::ContactConstraint *) {});
    }
  }
  /** Load collision managers */
  {
    auto config_collisions = config("collisions", std::vector<mc_rtc::Configuration>{});
    for(auto & config_cc : config_collisions)
    {
      if(!config_cc.has("type")) { config_cc.add("type", "collision"); }
      auto cc = mc_solver::ConstraintSetLoader::load<mc_solver::CollisionsConstraint>(solver(), config_cc);
      auto & r1 = robots().robot(cc->r1Index);
      auto & r2 = robots().robot(cc->r2Index);
      collision_constraints_[{r1.name(), r2.name()}] = cc;
      solver().addConstraintSet(*cc);
    }
  }
  /** Create contacts */
  if(config.has("contacts")) { contacts_ = config("contacts"); }
  contacts_changed_ = true;
  mc_rtc::log::info("MCController(base) ready");
}

MCController::~MCController()
{
  gui()->reset();
  datastore().clear();
}

mc_rbdyn::Robot & MCController::loadRobot(mc_rbdyn::RobotModulePtr rm, const std::string & name)
{
  // Load canonical robot model (for output and display)
  mc_rbdyn::RobotModulePtr canonicalModule = nullptr;
  const auto & cp = rm->canonicalParameters();
  if(cp == rm->parameters()) { canonicalModule = rm; }
  else
  {
    canonicalModule = mc_rbdyn::RobotLoader::get_robot_module(cp);
    if(!canonicalModule)
    {
      mc_rtc::log::critical("Failed to load the canonical module for {}, acting as if canonical was self", name);
      canonicalModule = rm;
    }
    else
    {
      if(!mc_rbdyn::check_module_compatibility(*rm, *canonicalModule))
      {
        mc_rtc::log::error_and_throw("Incompatibilities between a robot module and its canonical representation");
      }
    }
  }
  mc_rbdyn::LoadRobotParameters params{};
  auto & robot = loadRobot(rm, name, robots(), params);
  params.warn_on_missing_files(false).data(robot.data());
  loadRobot(rm, name, realRobots(), params);
  std::string urdf;
  auto loadUrdf = [&canonicalModule, &urdf]() -> const std::string &
  {
    if(urdf.size()) { return urdf; }
    const auto & urdfPath = canonicalModule->urdf_path;
    std::ifstream ifs(urdfPath);
    if(ifs.is_open())
    {
      std::stringstream urdfSS;
      urdfSS << ifs.rdbuf();
      urdf = urdfSS.str();
      return urdf;
    }
    mc_rtc::log::error("Could not open urdf file {} for robot {}, cannot initialize grippers", urdfPath,
                       canonicalModule->name);
    mc_rtc::log::error_and_throw("Failed to initialize grippers");
  };
  auto & outputRobot = loadRobot(canonicalModule, name, *outputRobots_, params);
  for(const auto & gripper : canonicalModule->grippers())
  {
    auto mimics = gripper.mimics();
    const auto & safety = gripper.safety() ? *gripper.safety() : canonicalModule->gripperSafety();
    if(mimics)
    {
      robot.data()->grippers[gripper.name].reset(
          new mc_control::Gripper(outputRobot, gripper.joints, *mimics, gripper.reverse_limits, safety));
    }
    else
    {
      robot.data()->grippers[gripper.name].reset(
          new mc_control::Gripper(outputRobot, gripper.joints, loadUrdf(), gripper.reverse_limits, safety));
    }
    robot.data()->grippersRef.push_back(std::ref(*robot.data()->grippers[gripper.name]));
  }
  loadRobot(canonicalModule, name, *outputRealRobots_, params);
  addRobotToLog(robot);
  addRobotToGUI(robot);
  if(solver().backend() == Backend::Tasks) { tasks_solver(solver()).updateNrVars(); }
  converters_.emplace_back(robot, outputRobot, robot.module().controlToCanonicalConfig);
  return robot;
}

mc_rbdyn::Robot & MCController::loadRobot(mc_rbdyn::RobotModulePtr rm,
                                          const std::string & name,
                                          mc_rbdyn::Robots & robots,
                                          const mc_rbdyn::LoadRobotParameters & params)
{
  assert(rm);
  auto & r = robots.load(name, *rm, params);
  r.mbc().gravity = mc_rtc::constants::gravity;
  r.forwardKinematics();
  r.forwardVelocity();
  return r;
}

void MCController::addRobotToGUI(const mc_rbdyn::Robot & r)
{
  if(!gui_) { return; }
  auto data = gui_->data();
  if(!data.has("robots")) { data.array("robots"); }
  if(!data.has("bodies")) { data.add("bodies"); }
  if(!data.has("surfaces")) { data.add("surfaces"); }
  if(!data.has("joints")) { data.add("joints"); }
  if(!data.has("frames")) { data.add("frames"); }
  data("robots").push(r.name());
  auto bs = data("bodies").array(r.name());
  for(const auto & b : r.mb().bodies()) { bs.push(b.name()); }
  data("surfaces").add(r.name(), r.availableSurfaces());
  data("joints").add(r.name(), r.module().ref_joint_order());
  data("frames").add(r.name(), r.frames());
  auto name = r.name();
  gui()->addElement({"Robots"}, mc_rtc::gui::Robot(r.name(), [name, this]() -> const mc_rbdyn::Robot &
                                                   { return this->outputRobot(name); }));
}

void MCController::addRobotToLog(const mc_rbdyn::Robot & r)
{
  // If this is the main robot we don't add a prefix
  const auto & name = r.name();
  auto prefix = robots().robot().name() == name ? "" : fmt::format("{}_", name);
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
    logger().addLogEntry(entry("qOut"),
                         [this, name, qOut]() mutable -> const std::vector<double> &
                         {
                           auto & robot = this->outputRobot(name);
                           for(size_t i = 0; i < qOut.size(); ++i)
                           {
                             auto mbcIndex = robot.jointIndexInMBC(i);
                             if(mbcIndex != -1) { qOut[i] = robot.mbc().q[static_cast<size_t>(mbcIndex)][0]; }
                           }
                           return qOut;
                         });
    auto & alphaOut = qOut;
    logger().addLogEntry(entry("alphaOut"),
                         [this, name, alphaOut]() mutable -> const std::vector<double> &
                         {
                           auto & robot = this->outputRobot(name);
                           for(size_t i = 0; i < alphaOut.size(); ++i)
                           {
                             auto mbcIndex = robot.jointIndexInMBC(i);
                             if(mbcIndex != -1) { alphaOut[i] = robot.mbc().alpha[static_cast<size_t>(mbcIndex)][0]; }
                           }
                           return alphaOut;
                         });
    auto & alphaDOut = qOut;
    logger().addLogEntry(entry("alphaDOut"),
                         [this, name, alphaDOut]() mutable -> const std::vector<double> &
                         {
                           auto & robot = this->outputRobot(name);
                           for(size_t i = 0; i < alphaDOut.size(); ++i)
                           {
                             auto mbcIndex = robot.jointIndexInMBC(i);
                             if(mbcIndex != -1) { alphaDOut[i] = robot.mbc().alphaD[static_cast<size_t>(mbcIndex)][0]; }
                           }
                           return alphaDOut;
                         });
    auto & tauOut = qOut;
    logger().addLogEntry(entry("tauOut"),
                         [this, name, tauOut]() mutable -> const std::vector<double> &
                         {
                           auto & robot = this->outputRobot(name);
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
                         [this, name]() -> const sva::PTransformd & { return outputRobot(name).mbc().bodyPosW[0]; });
    logger().addLogEntry(entry("ff_real"), [this, name]() -> const sva::PTransformd &
                         { return outputRealRobot(name).mbc().bodyPosW[0]; });
  }
  // Log all force sensors
  for(const auto & fs : robot(name).forceSensors())
  {
    const auto & fs_name = fs.name();
    logger().addLogEntry(entry_str(fs.name()), [this, name, fs_name]() -> const sva::ForceVecd &
                         { return robot(name).forceSensor(fs_name).wrench(); });
  }
  // Log all body sensors
  const auto & bodySensors = robot(name).bodySensors();
  for(size_t i = 0; i < bodySensors.size(); ++i)
  {
    const auto & bs_name = bodySensors[i].name();
    logger().addLogEntry(entry_str(bs_name + "_position"), [this, name, bs_name]() -> const Eigen::Vector3d &
                         { return robot(name).bodySensor(bs_name).position(); });
    logger().addLogEntry(entry_str(bs_name + "_orientation"), [this, name, bs_name]() -> const Eigen::Quaterniond &
                         { return robot(name).bodySensor(bs_name).orientation(); });
    logger().addLogEntry(entry_str(bs_name + "_linearVelocity"), [this, name, bs_name]() -> const Eigen::Vector3d &
                         { return robot(name).bodySensor(bs_name).linearVelocity(); });
    logger().addLogEntry(entry_str(bs_name + "_angularVelocity"), [this, name, bs_name]() -> const Eigen::Vector3d &
                         { return robot(name).bodySensor(bs_name).angularVelocity(); });
    logger().addLogEntry(entry_str(bs_name + "_linearAcceleration"), [this, name, bs_name]() -> const Eigen::Vector3d &
                         { return robot(name).bodySensor(bs_name).linearAcceleration(); });
    logger().addLogEntry(entry_str(bs_name + "_angularAcceleration"), [this, name, bs_name]() -> const Eigen::Vector3d &
                         { return robot(name).bodySensor(bs_name).angularAcceleration(); });
  }
  // Log all joint sensors
  for(const auto & js : robot(name).jointSensors())
  {
    const auto & jnt = js.joint();
    logger().addLogEntry(entry_str("JointSensor_" + jnt + "_motorTemperature"),
                         [this, name, jnt]() { return robot(name).jointJointSensor(jnt).motorTemperature(); });
    logger().addLogEntry(entry_str("JointSensor_" + jnt + "_driverTemperature"),
                         [this, name, jnt]() { return robot(name).jointJointSensor(jnt).driverTemperature(); });
    logger().addLogEntry(entry_str("JointSensor_" + jnt + "_motorCurrent"),
                         [this, name, jnt]() { return robot(name).jointJointSensor(jnt).motorCurrent(); });
    logger().addLogEntry(entry_str("JointSensor_" + jnt + "_motorStatus"),
                         [this, name, jnt]() { return robot(name).jointJointSensor(jnt).motorStatus(); });
  }
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
    for(const auto & fs : robot.forceSensors()) { logger().removeLogEntry(entry_str(fs.name())); }
    for(const auto & bs : robot.bodySensors())
    {
      logger().removeLogEntry(entry_str(bs.name() + "_position"));
      logger().removeLogEntry(entry_str(bs.name() + "_orientation"));
      logger().removeLogEntry(entry_str(bs.name() + "_linearVelocity"));
      logger().removeLogEntry(entry_str(bs.name() + "_angularVelocity"));
      logger().removeLogEntry(entry_str(bs.name() + "_linearAcceleration"));
      logger().removeLogEntry(entry_str(bs.name() + "_angularAcceleration"));
    }
    for(const auto & js : robot.jointSensors())
    {
      logger().removeLogEntry(entry_str("JointSensor_" + js.joint() + "_motorTemperature"));
      logger().removeLogEntry(entry_str("JointSensor_" + js.joint() + "_driverTemperature"));
      logger().removeLogEntry(entry_str("JointSensor_" + js.joint() + "_motorCurrent"));
      logger().removeLogEntry(entry_str("JointSensor_" + js.joint() + "_motorStatus"));
    }
    converters_.erase(converters_.begin() + robot.robotIndex());
  }
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
  outputRealRobots().removeRobot(name);
  outputRobots().removeRobot(name);
  realRobots().removeRobot(name);
  robots().removeRobot(name);
  if(solver().backend() == Backend::Tasks) { tasks_solver(solver()).updateNrVars(); }
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
    if(pipelineConfig("log", true)) { pipeline.addToLogger(logger()); }
    if(pipelineConfig("gui", false)) { pipeline.addToGUI(*gui()); }
  }
}

bool MCController::resetObserverPipelines()
{
  std::string desc;
  for(auto & pipeline : observerPipelines_)
  {
    pipeline.reset();
    if(desc.size()) { desc += "\n"; }
    desc += "- " + pipeline.desc();
  }
  if(desc.size()) { mc_rtc::log::success("[MCController::{}] State observation pipelines:\n{}", name_, desc); }
  return true;
}

bool MCController::runObserverPipelines()
{
  bool success = true;
  for(auto & pipeline : observerPipelines_) { success = pipeline.run() && success; }
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
  if(pipelineIt != observerPipelines_.end()) { return *pipelineIt; }
  else { mc_rtc::log::error_and_throw("Observer pipeline {} does not exist", name); }
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
  auto startUpdateContacts = mc_rtc::clock::now();
  updateContacts();
  updateContacts_dt_ = mc_rtc::clock::now() - startUpdateContacts;
  if(!qpsolver->run(fType))
  {
    mc_rtc::log::error("QP failed to run()");
    return false;
  }
  return true;
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
  updateContacts();
  if(gui_)
  {
    gui_->removeElement({"Contacts"}, "Contacts");
    gui_->addElement({"Contacts"}, mc_rtc::gui::Table("Contacts", {"R1", "S1", "R2", "S2", "DoF", "Friction"},
                                                      [this]() -> const std::vector<ContactTableDataT> &
                                                      { return contacts_table_; }));
    gui_->removeElement({"Contacts", "Add"}, "Add contact");
    gui_->addElement({"Contacts", "Add"},
                     mc_rtc::gui::Form(
                         "Add contact",
                         [this](const mc_rtc::Configuration & data)
                         {
                           std::string r0 = data("R0");
                           std::string r1 = data("R1");
                           std::string r0Surface = data("R0 surface");
                           std::string r1Surface = data("R1 surface");
                           double friction = data("Friction", mc_rbdyn::Contact::defaultFriction);
                           Eigen::Vector6d dof = data("dof", Eigen::Vector6d::Ones().eval());
                           addContact({r0, r1, r0Surface, r1Surface, friction, dof});
                         },
                         mc_rtc::gui::FormDataComboInput{"R0", true, {"robots"}},
                         mc_rtc::gui::FormDataComboInput{"R0 surface", true, {"surfaces", "$R0"}},
                         mc_rtc::gui::FormDataComboInput{"R1", true, {"robots"}},
                         mc_rtc::gui::FormDataComboInput{"R1 surface", true, {"surfaces", "$R1"}},
                         mc_rtc::gui::FormNumberInput("Friction", false, mc_rbdyn::Contact::defaultFriction),
                         mc_rtc::gui::FormArrayInput<Eigen::Vector6d>("dof", false, Eigen::Vector6d::Ones())));
  }
  logger().addLogEntry("perf_UpdateContacts", [this]() { return updateContacts_dt_.count(); }, true);
}

void MCController::updateContacts()
{
  if(contacts_changed_ && contact_constraint_)
  {
    std::vector<mc_rbdyn::Contact> contacts;
    if(solver().backend() == Backend::Tasks) { contact_constraint_->contactConstr()->resetDofContacts(); }

    auto ensureValidContact = [this](const std::string & robotName, const std::string & surfaceName)
    {
      if(!hasRobot(robotName))
      {
        const auto availableRobots =
            mc_rtc::io::to_string(robots(), [](const mc_rbdyn::Robot & r) { return r.name(); });
        mc_rtc::log::error_and_throw("Failed to add contact: no robot named {} (available robots: {})", robotName,
                                     availableRobots);
      }
      if(!robot(robotName).hasSurface(surfaceName))
      {
        mc_rtc::log::error_and_throw("Failed to add contact: no surface named {} in robot {}", surfaceName, robotName);
      }
    };
    contacts_table_.resize(contacts_.size());
    size_t table_idx = 0;
    for(const auto & c : contacts_)
    {
      const auto & r1 = c.r1.has_value() ? c.r1.value() : robot().name();
      const auto & r2 = c.r2.has_value() ? c.r2.value() : robot().name();
      ensureValidContact(r1, c.r1Surface);
      ensureValidContact(r2, c.r2Surface);
      auto r1Index = robot(r1).robotIndex();
      auto r2Index = robot(r2).robotIndex();
      contacts.emplace_back(robots(), r1Index, r2Index, c.r1Surface, c.r2Surface, c.friction);
      contacts.back().dof(c.dof);
      if(solver().backend() == Backend::Tasks)
      {
        auto cId = contacts.back().contactId(robots());
        contact_constraint_->contactConstr()->addDofContact(cId, c.dof.asDiagonal());
      }
      auto & table_data = contacts_table_[table_idx];
      std::get<0>(table_data) = r1;
      std::get<1>(table_data) = c.r1Surface;
      std::get<2>(table_data) = r2;
      std::get<3>(table_data) = c.r2Surface;
      std::get<4>(table_data) = fmt::format("{}", MC_FMT_STREAMED(c.dof.transpose()));
      std::get<5>(table_data) = c.friction;
      table_idx++;
    }
    solver().setContacts(mc_solver::QPSolver::ControllerToken{}, contacts);
    if(gui_)
    {
      gui_->removeCategory({"Contacts", "Remove"});
      for(const auto & c : contacts_)
      {
        const auto & r1 = c.r1.has_value() ? c.r1.value() : robot().name();
        const auto & r2 = c.r2.has_value() ? c.r2.value() : robot().name();
        std::string bName = r1 + "::" + c.r1Surface + " & " + r2 + "::" + c.r2Surface;
        gui_->addElement({"Contacts", "Remove"}, mc_rtc::gui::Button(bName, [this, &c]() { removeContact(c); }));
      }
    }
    if(solver().backend() == Backend::Tasks) { contact_constraint_->contactConstr()->updateDofContacts(); }
  }
  contacts_changed_ = false;
}

void MCController::addCollisions(const std::string & r1,
                                 const std::string & r2,
                                 const std::vector<mc_rbdyn::Collision> & collisions)
{
  if(r1 != r2 && collision_constraints_.count({r2, r1}))
  {
    std::vector<mc_rbdyn::Collision> swapped;
    swapped.reserve(collisions.size());
    for(const auto & c : collisions) { swapped.push_back({c.body2, c.body1, c.iDist, c.sDist, c.damping}); }
    addCollisions(r2, r1, swapped);
    return;
  }
  if(!collision_constraints_.count({r1, r2}))
  {
    if(!hasRobot(r1) || !hasRobot(r2))
    {
      mc_rtc::log::error("Try to add collision for robot {} and {} which are not involved in this controller", r1, r2);
      return;
    }
    auto r1Index = robot(r1).robotIndex();
    auto r2Index = robot(r2).robotIndex();
    collision_constraints_[{r1, r2}] =
        std::make_shared<mc_solver::CollisionsConstraint>(robots(), r1Index, r2Index, solver().dt());
    solver().addConstraintSet(*collision_constraints_[{r1, r2}]);
  }
  auto & cc = collision_constraints_[{r1, r2}];
  mc_rtc::log::info("Add collisions {}/{}", r1, r2);
  for(const auto & c : collisions) { mc_rtc::log::info("- {}::{}/{}::{}", r1, c.body1, r2, c.body2); }
  cc->addCollisions(solver(), collisions);
}

bool MCController::hasCollision(const std::string & r1,
                                const std::string & r2,
                                const mc_rbdyn::Collision & col) const noexcept
{
  return hasCollision(r1, r2, col.body1, col.body2);
}

bool MCController::hasCollision(const std::string & r1,
                                const std::string & r2,
                                const std::string & c1,
                                const std::string & c2) const noexcept
{
  auto it = collision_constraints_.find({r1, r2});
  if(it != collision_constraints_.end()) { return it->second->hasCollision(c1, c2); }
  if(r1 != r2)
  {
    it = collision_constraints_.find({r2, r1});
    if(it != collision_constraints_.end()) { return it->second->hasCollision(c2, c1); }
  }
  return false;
}

void MCController::removeCollisions(const std::string & r1,
                                    const std::string & r2,
                                    const std::vector<mc_rbdyn::Collision> & collisions)
{
  if(!collision_constraints_.count({r1, r2})) { return; }
  auto & cc = collision_constraints_[{r1, r2}];
  mc_rtc::log::info("Remove collisions {}/{}", r1, r2);
  for(const auto & c : collisions) { mc_rtc::log::info("- {}::{}/{}::{}", r1, c.body1, r2, c.body2); }
  cc->removeCollisions(solver(), collisions);
}

void MCController::removeCollisions(const std::string & r1, const std::string & r2)
{
  if(!collision_constraints_.count({r1, r2})) { return; }
  auto & cc = collision_constraints_[{r1, r2}];
  mc_rtc::log::info("Remove all collisions {}/{}", r1, r2);
  cc->reset();
}

void MCController::addContact(const Contact & c)
{
  bool inserted;
  ContactSet::iterator it;
  std::tie(it, inserted) = contacts_.insert(c);
  contacts_changed_ |= inserted;
  const auto & r1 = c.r1.has_value() ? c.r1.value() : robot().name();
  const auto & r2 = c.r2.has_value() ? c.r2.value() : robot().name();
  if(!inserted)
  {
    if(it->dof != c.dof)
    {
      mc_rtc::log::info("Changed contact DoF {}::{}/{}::{} to {}", r1, c.r1Surface, r2, c.r2Surface,
                        MC_FMT_STREAMED(c.dof.transpose()));
      it->dof = c.dof;
      contacts_changed_ = true;
    }
    if(it->friction != c.friction)
    {
      mc_rtc::log::info("Changed contact friction {}::{}/{}::{} to {}", r1, c.r1Surface, r2, c.r2Surface, c.friction);
      it->friction = c.friction;
      contacts_changed_ = true;
    }
  }
  else
  {
    mc_rtc::log::info("Add contact {}::{}/{}::{} (DoF: {})", r1, c.r1Surface, r2, c.r2Surface,
                      MC_FMT_STREAMED(c.dof.transpose()));
  }
}

void MCController::removeContact(const Contact & c)
{
  contacts_changed_ |= static_cast<bool>(contacts_.erase(c));
  if(contacts_changed_)
  {
    const auto & r1 = c.r1.has_value() ? c.r1.value() : robot().name();
    const auto & r2 = c.r2.has_value() ? c.r2.value() : robot().name();
    mc_rtc::log::info("Remove contact {}::{}/{}::{}", r1, c.r1Surface, r2, c.r2Surface);
  }
}

void MCController::clearContacts()
{
  contacts_changed_ = contacts_.size() != 0;
  contacts_.clear();
}

const ContactSet & MCController::contacts() const
{
  return contacts_;
}

bool MCController::hasContact(const Contact & c) const
{
  return std::find(contacts_.begin(), contacts_.end(), c) != contacts_.end();
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

mc_rtc::Configuration MCController::robot_config(const mc_rbdyn::Robot & robot) const
{
  return robot_config(robot.module().name);
}

mc_rtc::Configuration MCController::robot_config(const std::string & robot) const
{
  mc_rtc::Configuration result;
  bfs::path system_path = bfs::path(loading_location_) / this->name_ / (robot + ".conf");
  bfs::path user_path = mc_rtc::user_config_directory_path("controllers");
  user_path = user_path / name_ / (robot + ".conf");
  auto load_conf = [&result](const std::string & path)
  {
    result.load(path);
    mc_rtc::log::info("Controller's robot configuration loaded from {}", path);
  };
  auto load_conf_or_yaml = [&load_conf](bfs::path & in)
  {
    if(bfs::exists(in)) { return load_conf(in.string()); }
    in.replace_extension(".yaml");
    if(bfs::exists(in)) { return load_conf(in.string()); }
    in.replace_extension(".yml");
    if(bfs::exists(in)) { return load_conf(in.string()); }
  };
  load_conf_or_yaml(system_path);
  load_conf_or_yaml(user_path);
  return result;
}

} // namespace mc_control
