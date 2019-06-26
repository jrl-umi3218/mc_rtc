/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/fsm/Controller.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/configuration_io.h>
#include <mc_solver/ConstraintSetLoader.h>
#include <mc_tasks/MetaTaskLoader.h>

namespace mc_rtc
{

mc_control::fsm::Contact ConfigurationLoader<mc_control::fsm::Contact>::load(const mc_rtc::Configuration & config)
{
  return mc_control::fsm::Contact(config("r1"), config("r2"), config("r1Surface"), config("r2Surface"),
                                  config("dof", Eigen::Vector6d::Ones().eval()));
}

} // namespace mc_rtc

namespace mc_control
{

namespace fsm
{

Contact Contact::from_mc_rbdyn(const Controller & ctl, const mc_rbdyn::Contact & contact)
{
  return {ctl.robots().robot(contact.r1Index()).name(), ctl.robots().robot(contact.r2Index()).name(),
          contact.r1Surface()->name(), contact.r2Surface()->name()};
}

Controller::Controller(std::shared_ptr<mc_rbdyn::RobotModule> rm, double dt, const mc_rtc::Configuration & config)
: MCController(std::vector<mc_rbdyn::RobotModulePtr>{rm}, dt), config_(config),
  factory_(config("StatesLibraries", std::vector<std::string>{}),
           config("StatesFiles", std::vector<std::string>{}),
           config("VerboseStateFactory", false))
{
  idle_keep_state_ = config("IdleKeepState", false);
  robots_idx_[robot().name()] = 0;
  /** Load additional robots from the configuration */
  {
    auto config_robots = config("robots", std::map<std::string, mc_rtc::Configuration>{});
    for(const auto & cr : config_robots)
    {
      const auto & name = cr.first;
      if(robots_idx_.count(name))
      {
        LOG_ERROR_AND_THROW(std::runtime_error, "FSM controller cannot have two robots with the same name")
      }
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
        LOG_ERROR_AND_THROW(std::runtime_error,
                            "FSM controller only handles robot modules that require two parameters at most")
      }
      if(!rm)
      {
        LOG_ERROR_AND_THROW(std::runtime_error, "Failed to load " << name << " as specified in configuration");
      }
      auto & r = loadRobot(rm, name);
      robots_idx_[name] = r.robotIndex();
    }
    LOG_INFO("Robots loaded in FSM controller:")
    for(const auto & r : robots())
    {
      LOG_INFO("- " << r.name())
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
      LOG_WARNING("No contact constraint loaded from the FSM configuration")
    }
  }
  /** Load collision managers */
  {
    auto config_collisions = config("collisions", std::vector<mc_rtc::Configuration>{});
    for(const auto & config_cc : config_collisions)
    {
      auto cc = mc_solver::ConstraintSetLoader::load<mc_solver::CollisionsConstraint>(solver(), config_cc);
      auto & r1 = robots().robot(cc->r1Index);
      auto & r2 = robots().robot(cc->r2Index);
      collision_constraints_[{r1.name(), r2.name()}] = cc;
      solver().addConstraintSet(*cc);
    }
  }
  /** Create posture task for actuated robots */
  for(auto & robot : robots())
  {
    if(robot.mb().nrDof() - robot.mb().joint(0).dof() > 0)
    {
      double stiffness = 1.0;
      double weight = 10.0;
      if(config.has(robot.name()))
      {
        auto robot_config = config(robot.name());
        if(robot_config.has("posture"))
        {
          robot_config("posture")("stiffness", stiffness);
          robot_config("posture")("weight", weight);
        }
      }
      auto t = std::make_shared<mc_tasks::PostureTask>(solver(), robot.robotIndex(), stiffness, weight);
      t->name("FSM_" + t->name());
      posture_tasks_[robot.name()] = t;
      solver().addTask(t);
    }
    if(robot.mb().joint(0).type() == rbd::Joint::Free)
    {
      double stiffness = 2.0;
      double weight = 100.0;
      if(config.has(robot.name()))
      {
        auto robot_config = config(robot.name());
        if(robot_config.has("ff"))
        {
          robot_config("ff")("stiffness", stiffness);
          robot_config("ff")("weight", weight);
        }
      }
      auto t = std::make_shared<mc_tasks::EndEffectorTask>(robot.mb().body(0).name(), solver().robots(),
                                                           robot.robotIndex(), stiffness, weight);
      t->name("FSM_" + t->name());
      ff_tasks_[robot.name()] = t;
    }
  }
  /** Create contacts */
  contacts_ = config("contacts", ContactSet{});
  contacts_changed_ = true;
  /** Load more states if they are provided in the configuration */
  if(config.has("states"))
  {
    factory_.load(config("states"));
  }
  /** Setup executor */
  executor_.init(*this, config_);
  /** Setup initial pos */
  config("init_pos", init_pos_);
  if(init_pos_.size())
  {
    if(init_pos_.size() != 7)
    {
      LOG_ERROR("Stored init_pos is not of size 7")
      LOG_WARNING("Using default position")
      init_pos_.resize(0);
    }
  }
}

bool Controller::run()
{
  return run(mc_solver::FeedbackType::None);
}

bool Controller::run(mc_solver::FeedbackType fType)
{
  if(robots().size() != robots_idx_.size())
  {
    for(const auto & r : robots())
    {
      robots_idx_[r.name()] = r.robotIndex();
    }
  }
  if(contacts_changed_)
  {
    std::vector<mc_rbdyn::Contact> contacts;
    contact_constraint_->contactConstr->resetDofContacts();
    for(const auto & c : contacts_)
    {
      contacts.emplace_back(robots(), static_cast<unsigned int>(robots_idx_.at(c.r1)),
                            static_cast<unsigned int>(robots_idx_.at(c.r2)), c.r1Surface, c.r2Surface);
      auto cId = contacts.back().contactId(robots());
      contact_constraint_->contactConstr->addDofContact(cId, c.dof.asDiagonal());
    }
    solver().setContacts(contacts);
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
    contacts_changed_ = false;
  }
  executor_.run(*this, idle_keep_state_);
  if(!executor_.running())
  {
    if(running_)
    {
      running_ = false;
      startIdleState();
    }
  }
  else
  {
    if(!running_)
    {
      running_ = true;
      teardownIdleState();
    }
  }
  return MCController::run(fType);
}

void Controller::reset(const ControllerResetData & data)
{
  auto q = data.q;
  if(init_pos_.size())
  {
    q[0] = init_pos_;
  }
  MCController::reset({q});
  /** GUI information */
  if(gui_)
  {
    auto all_states = factory_.states();
    std::sort(all_states.begin(), all_states.end());
    gui_->data().add("states", all_states);
    gui_->removeElement({"FSM"}, "Contacts");
    gui_->addElement({"FSM"}, mc_rtc::gui::Label("Contacts", [this]() {
                       std::string ret;
                       for(const auto & c : contacts_)
                       {
                         std::stringstream ss;
                         ss << c.r1Surface << "/" << c.r2Surface << " | " << c.dof.transpose() << "\n";
                         ret += ss.str();
                       }
                       if(ret.size())
                       {
                         ret.pop_back();
                       }
                       return ret;
                     }));
    gui_->removeElement({"Contacts", "Add"}, "Add contact");
    gui_->addElement(
        {"Contacts", "Add"},
        mc_rtc::gui::Form("Add contact",
                          [this](const mc_rtc::Configuration & data) {
                            std::string r0 = data("R0");
                            std::string r1 = data("R1");
                            std::string r0Surface = data("R0 surface");
                            std::string r1Surface = data("R1 surface");
                            Eigen::Vector6d dof = data("dof", Eigen::Vector6d::Ones().eval());
                            addContact({r0, r1, r0Surface, r1Surface, dof});
                          },
                          mc_rtc::gui::FormDataComboInput{"R0", true, {"robots"}},
                          mc_rtc::gui::FormDataComboInput{"R0 surface", true, {"surfaces", "$R0"}},
                          mc_rtc::gui::FormDataComboInput{"R1", true, {"robots"}},
                          mc_rtc::gui::FormDataComboInput{"R1 surface", true, {"surfaces", "$R1"}},
                          mc_rtc::gui::FormArrayInput<Eigen::Vector6d>{"dof", false, Eigen::Vector6d::Ones()}));
  }
  startIdleState();
}

void Controller::resetPostures()
{
  for(auto & pt : posture_tasks_)
  {
    pt.second->reset();
  }
}

void Controller::startIdleState()
{
  resetPostures();
  for(auto & fft : ff_tasks_)
  {
    fft.second->reset();
    solver().addTask(fft.second);
  }
}

void Controller::teardownIdleState()
{
  for(auto & fft : ff_tasks_)
  {
    solver().removeTask(fft.second);
  }
}

bool Controller::play_next_stance()
{
  return executor_.next();
}

bool Controller::read_msg(std::string & msg)
{
  std::string token;
  std::stringstream ss;
  ss << msg;
  ss >> token;
  if(token == "interrupt")
  {
    interrupt();
    return true;
  }
  if(token == "play")
  {
    std::string state;
    ss >> state;
    return resume(state);
  }
  return executor_.read_msg(msg) || MCController::read_msg(msg);
}

bool Controller::read_write_msg(std::string & msg, std::string & out)
{
  std::string token;
  std::stringstream ss;
  ss << msg;
  ss >> token;
  if(token == "current_state")
  {
    out = executor_.state();
    return true;
  }
  if(token == "output")
  {
    out = executor_.output();
    return true;
  }
  if(token == "running")
  {
    out = executor_.running() ? "1" : "0";
    return true;
  }
  if(token == "ready")
  {
    out = executor_.ready() ? "1" : "0";
    return true;
  }
  return executor_.read_write_msg(msg, out) || MCController::read_write_msg(msg, out);
}

void Controller::addCollisions(const std::string & r1,
                               const std::string & r2,
                               const std::vector<mc_rbdyn::Collision> & collisions)
{
  if(!collision_constraints_.count({r1, r2}))
  {
    if(robots_idx_.count(r1) * robots_idx_.count(r2) == 0)
    {
      LOG_ERROR("Try to add collision for robot " << r1 << " and " << r2 << " which are not involved in this FSM")
      return;
    }
    collision_constraints_[{r1, r2}] =
        std::make_shared<mc_solver::CollisionsConstraint>(robots(), static_cast<unsigned int>(robots_idx_[r1]),
                                                          static_cast<unsigned int>(robots_idx_[r2]), solver().dt());
    solver().addConstraintSet(*collision_constraints_[{r1, r2}]);
  }
  auto & cc = collision_constraints_[{r1, r2}];
  LOG_INFO("[FSM] Add collisions " << r1 << "/" << r2)
  for(const auto & c : collisions)
  {
    LOG_INFO("[FSM] - " << r1 << "::" << c.body1 << "/" << r2 << "::" << c.body2)
  }
  cc->addCollisions(solver(), collisions);
}

void Controller::removeCollisions(const std::string & r1,
                                  const std::string & r2,
                                  const std::vector<mc_rbdyn::Collision> & collisions)
{
  if(!collision_constraints_.count({r1, r2}))
  {
    return;
  }
  auto & cc = collision_constraints_[{r1, r2}];
  LOG_INFO("[FSM] Remove collisions " << r1 << "/" << r2)
  for(const auto & c : collisions)
  {
    LOG_INFO("[FSM] - " << r1 << "::" << c.body1 << "/" << r2 << "::" << c.body2)
  }
  cc->removeCollisions(solver(), collisions);
}

void Controller::removeCollisions(const std::string & r1, const std::string & r2)
{
  if(!collision_constraints_.count({r1, r2}))
  {
    return;
  }
  auto & cc = collision_constraints_[{r1, r2}];
  LOG_INFO("[FSM] Remove all collisions " << r1 << "/" << r2)
  cc->reset();
}

bool Controller::hasRobot(const std::string & robot) const
{
  return robots_idx_.count(robot) != 0;
}

mc_rbdyn::Robot & Controller::robot(const std::string & name)
{
  return solver().robot(static_cast<unsigned int>(robots_idx_.at(name)));
}

std::shared_ptr<mc_tasks::PostureTask> Controller::getPostureTask(const std::string & robot)
{
  if(posture_tasks_.count(robot))
  {
    return posture_tasks_.at(robot);
  }
  return nullptr;
}

void Controller::addContact(const Contact & c)
{
  bool inserted;
  std::set<Contact>::iterator it;
  std::tie(it, inserted) = contacts_.insert(c);
  contacts_changed_ |= inserted;
  if(!inserted)
  {
    if(it->dof != c.dof)
    {
      LOG_INFO("[FSM] Changed contact DoF " << c.r1 << "::" << c.r1Surface << "/" << c.r2 << "::" << c.r2Surface)
      it->dof = c.dof;
      contacts_changed_ = true;
    }
  }
  else
  {
    LOG_INFO("[FSM] Add contact " << c.r1 << "::" << c.r1Surface << "/" << c.r2 << "::" << c.r2Surface)
  }
}

void Controller::removeContact(const Contact & c)
{
  contacts_changed_ |= static_cast<bool>(contacts_.erase(c));
  if(contacts_changed_)
  {
    LOG_INFO("[FSM] Remove contact " << c.r1 << "::" << c.r1Surface << "/" << c.r2 << "::" << c.r2Surface)
  }
}

const ContactSet & Controller::contacts() const
{
  return contacts_;
}

bool Controller::hasContact(const Contact & c) const
{
  for(const auto & co : contacts_)
  {
    if(co == c)
    {
      return true;
    }
  }
  return false;
}

bool Controller::set_joint_pos(const std::string & jname, const double & pos)
{
  if(robot().hasJoint(jname))
  {
    getPostureTask(robot().name())->target({{jname, {pos}}});
    return true;
  }
  return false;
}

bool Controller::resume(const std::string & state)
{
  if(!factory_.hasState(state))
  {
    LOG_ERROR("Cannot play unloaded state: " << state)
    return false;
  }
  return executor_.resume(state);
}

} // namespace fsm

} // namespace mc_control
