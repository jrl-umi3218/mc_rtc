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

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_solver/ConstraintSetLoader.h>

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <array>
#include <fstream>

namespace mc_rtc
{

mc_control::Contact ConfigurationLoader<mc_control::Contact>::load(const mc_rtc::Configuration & config)
{
  return mc_control::Contact(config("r1"), config("r2"), config("r1Surface"), config("r2Surface"),
                             config("friction", mc_rbdyn::Contact::defaultFriction),
                             config("dof", Eigen::Vector6d::Ones().eval()));
}

} // namespace mc_rtc

namespace mc_control
{

Contact Contact::from_mc_rbdyn(const MCController & ctl, const mc_rbdyn::Contact & contact)
{

  Eigen::Vector6d dof = Eigen::Vector6d::Ones();
  const auto cId = contact.contactId(ctl.robots());
  if(ctl.contactConstraint.contactConstr->hasDoFContact(cId))
  {
    dof = ctl.contactConstraint.contactConstr->dofContact(cId).diagonal();
  }

  return {ctl.robots().robot(contact.r1Index()).name(),
          ctl.robots().robot(contact.r2Index()).name(),
          contact.r1Surface()->name(),
          contact.r2Surface()->name(),
          contact.friction(),
          dof};
}

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
  qpsolver->controller(this);
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
  /** Load additional robots from the configuration */
  {
    auto init_robot = [&](const std::string & robotName, const mc_rtc::Configuration & config) {
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
    auto config_robots = config("robots", std::map<std::string, mc_rtc::Configuration>{});
    for(const auto & cr : config_robots)
    {
      if(cr.first == robot().name())
      {
        if(config.has("init_pos") && cr.second.has("init_pos"))
        {
          mc_rtc::log::error_and_throw("You have both a global \"init_pos\" entry and a \"robots/{}/init_pos\" entry "
                                       "in your FSM configuration. Please use \"robots/{}/init_pos\" only.",
                                       robot().name(), robot().name());
        }
        init_robot(cr.first, cr.second);
      }
      else if(!cr.second.has("module"))
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
        const auto & name = cr.first;
        std::string module = cr.second("module");
        auto params = cr.second("params", std::vector<std::string>{});
        mc_rbdyn::RobotModulePtr rm = nullptr;
        if(params.size() == 0)
        {
          rm = mc_rbdyn::RobotLoader::get_robot_module(module);
        }
        else if(params.size() == 1)
        {
          rm = mc_rbdyn::RobotLoader::get_robot_module(module, params.at(0));
        }
        else if(params.size() == 2)
        {
          rm = mc_rbdyn::RobotLoader::get_robot_module(module, params.at(0), params.at(1));
        }
        else
        {
          mc_rtc::log::error_and_throw("Controller only handles robot modules that require two parameters at most");
        }
        if(!rm)
        {
          mc_rtc::log::error_and_throw("Failed to load {} as specified in configuration", name);
        }
        loadRobot(rm, name);
        init_robot(name, cr.second);
      }
    }
    mc_rtc::log::info("Robots loaded in controller:");
    for(const auto & r : robots())
    {
      mc_rtc::log::info("- {}", r.name());
    }
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
          std::shared_ptr<mc_solver::ContactConstraint>(&contactConstraint, [](mc_solver::ContactConstraint *) {});
    }
  }
  /** Load collision managers */
  {
    auto config_collisions = config("collisions", std::vector<mc_rtc::Configuration>{});
    for(auto & config_cc : config_collisions)
    {
      if(!config_cc.has("type"))
      {
        config_cc.add("type", "collision");
      }
      auto cc = mc_solver::ConstraintSetLoader::load<mc_solver::CollisionsConstraint>(solver(), config_cc);
      auto & r1 = robots().robot(cc->r1Index);
      auto & r2 = robots().robot(cc->r2Index);
      collision_constraints_[{r1.name(), r2.name()}] = cc;
      solver().addConstraintSet(*cc);
    }
  }
  /** Create contacts */
  if(config.has("contacts"))
  {
    contacts_ = config("contacts");
  }
  contacts_changed_ = true;
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
    if(!data.has("frames"))
    {
      data.add("frames");
    }
    data("robots").push(r.name());
    auto bs = data("bodies").array(r.name());
    for(const auto & b : r.mb().bodies())
    {
      bs.push(b.name());
    }
    data("surfaces").add(r.name(), r.availableSurfaces());
    data("joints").add(r.name(), r.module().ref_joint_order());
    data("frames").add(r.name(), r.frames());
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
      logger().addLogEntry(entry_str(bs_name + "_position"), [this, name, bs_name]() -> const Eigen::Vector3d & {
        return robot(name).bodySensor(bs_name).position();
      });
      logger().addLogEntry(entry_str(bs_name + "_orientation"), [this, name, bs_name]() -> const Eigen::Quaterniond & {
        return robot(name).bodySensor(bs_name).orientation();
      });
      logger().addLogEntry(entry_str(bs_name + "_linearVelocity"), [this, name, bs_name]() -> const Eigen::Vector3d & {
        return robot(name).bodySensor(bs_name).linearVelocity();
      });
      logger().addLogEntry(entry_str(bs_name + "_angularVelocity"), [this, name, bs_name]() -> const Eigen::Vector3d & {
        return robot(name).bodySensor(bs_name).angularVelocity();
      });
      logger().addLogEntry(entry_str(bs_name + "_linearAcceleration"),
                           [this, name, bs_name]() -> const Eigen::Vector3d & {
                             return robot(name).bodySensor(bs_name).linearAcceleration();
                           });
      logger().addLogEntry(entry_str(bs_name + "_angularAcceleration"),
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
  updateContacts();
  if(gui_)
  {
    gui_->addElement({"Contacts"}, mc_rtc::gui::Table(
                                       "Contacts", {"R1", "S1", "R2", "S2", "DoF", "Friction"},
                                       [this]() -> const std::vector<ContactTableDataT> & { return contacts_table_; }));
    gui_->addElement({"Contacts", "Add"},
                     mc_rtc::gui::Form(
                         "Add contact",
                         [this](const mc_rtc::Configuration & data) {
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
  logger().addLogEntry("perf_UpdateContacts", [this]() { return updateContacts_dt_.count(); });
}

void MCController::updateContacts()
{
  if(contacts_changed_ && contact_constraint_)
  {
    std::vector<mc_rbdyn::Contact> contacts;
    contact_constraint_->contactConstr->resetDofContacts();

    auto ensureValidContact = [this](const std::string & robotName, const std::string & surfaceName) {
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
      ensureValidContact(c.r1, c.r1Surface);
      ensureValidContact(c.r2, c.r2Surface);
      auto r1Index = robot(c.r1).robotIndex();
      auto r2Index = robot(c.r2).robotIndex();
      contacts.emplace_back(robots(), r1Index, r2Index, c.r1Surface, c.r2Surface, c.friction);
      auto cId = contacts.back().contactId(robots());
      contact_constraint_->contactConstr->addDofContact(cId, c.dof.asDiagonal());
      auto & table_data = contacts_table_[table_idx];
      std::get<0>(table_data) = c.r1;
      std::get<1>(table_data) = c.r1Surface;
      std::get<2>(table_data) = c.r2;
      std::get<3>(table_data) = c.r2Surface;
      std::get<4>(table_data) = fmt::format("{}", c.dof.transpose());
      std::get<5>(table_data) = c.friction;
      table_idx++;
    }
    solver().setContacts(mc_solver::QPSolver::ControllerToken{}, contacts);
    if(gui_)
    {
      gui_->removeCategory({"Contacts", "Remove"});
      for(const auto & c : contacts_)
      {
        std::string bName = c.r1 + "::" + c.r1Surface + " & " + c.r2 + "::" + c.r2Surface;
        gui_->addElement({"Contacts", "Remove"}, mc_rtc::gui::Button(bName, [this, &c]() { removeContact(c); }));
      }
    }
    contact_constraint_->contactConstr->updateDofContacts();
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
    for(const auto & c : collisions)
    {
      swapped.push_back({c.body2, c.body1, c.iDist, c.sDist, c.damping});
    }
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
  for(const auto & c : collisions)
  {
    mc_rtc::log::info("- {}::{}/{}::{}", r1, c.body1, r2, c.body2);
  }
  cc->addCollisions(solver(), collisions);
}

void MCController::removeCollisions(const std::string & r1,
                                    const std::string & r2,
                                    const std::vector<mc_rbdyn::Collision> & collisions)
{
  if(!collision_constraints_.count({r1, r2}))
  {
    return;
  }
  auto & cc = collision_constraints_[{r1, r2}];
  mc_rtc::log::info("Remove collisions {}/{}", r1, r2);
  for(const auto & c : collisions)
  {
    mc_rtc::log::info("- {}::{}/{}::{}", r1, c.body1, r2, c.body2);
  }
  cc->removeCollisions(solver(), collisions);
}

void MCController::removeCollisions(const std::string & r1, const std::string & r2)
{
  if(!collision_constraints_.count({r1, r2}))
  {
    return;
  }
  auto & cc = collision_constraints_[{r1, r2}];
  mc_rtc::log::info("Remove all collisions {}/{}", r1, r2);
  cc->reset();
}

bool MCController::hasRobot(const std::string & robot) const
{
  return robots().hasRobot(robot);
}

void MCController::addContact(const Contact & c)
{
  bool inserted;
  ContactSet::iterator it;
  std::tie(it, inserted) = contacts_.insert(c);
  contacts_changed_ |= inserted;
  if(!inserted)
  {
    if(it->dof != c.dof)
    {
      mc_rtc::log::info("Changed contact DoF {}::{}/{}::{} to {}", c.r1, c.r1Surface, c.r2, c.r2Surface,
                        c.dof.transpose());
      it->dof = c.dof;
      contacts_changed_ = true;
    }
    if(it->friction != c.friction)
    {
      mc_rtc::log::info("Changed contact friction {}::{}/{}::{} to {}", c.r1, c.r1Surface, c.r2, c.r2Surface,
                        c.friction);
      it->friction = c.friction;
      contacts_changed_ = true;
    }
  }
  else
  {
    mc_rtc::log::info("Add contact {}::{}/{}::{} (DoF: {})", c.r1, c.r1Surface, c.r2, c.r2Surface, c.dof.transpose());
  }
}

void MCController::removeContact(const Contact & c)
{
  contacts_changed_ |= static_cast<bool>(contacts_.erase(c));
  if(contacts_changed_)
  {
    mc_rtc::log::info("Remove contact {}::{}/{}::{}", c.r1, c.r1Surface, c.r2, c.r2Surface);
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
