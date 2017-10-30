#include <mc_control/mc_fsm_controller.h>

#include <mc_rbdyn/configuration_io.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_solver/ConstraintSetLoader.h>
#include <mc_tasks/MetaTaskLoader.h>

namespace mc_rtc
{

template<>
struct ConfigurationLoader<mc_control::FSMContact>
{
  static mc_control::FSMContact load(const mc_rtc::Configuration & config)
  {
    mc_control::FSMContact ret;
    ret.r1 = static_cast<std::string>(config("r1"));
    ret.r1Surface = static_cast<std::string>(config("r1Surface"));
    ret.r2 = static_cast<std::string>(config("r2"));
    ret.r2Surface = static_cast<std::string>(config("r2Surface"));
    return ret;
  }
};

}

namespace mc_control
{

FSMContact FSMContact::from_mc_rbdyn(const FSMController & ctl, const mc_rbdyn::Contact & contact)
{
  return {
    ctl.robots().robot(contact.r1Index()).name(),
    ctl.robots().robot(contact.r2Index()).name(),
    contact.r1Surface()->name(),
    contact.r2Surface()->name()
  };
}

std::pair<bool, FSMController::Transition>
  FSMController::TransitionMap::transition(const std::string & state,
                                           const std::string & output) const
{
  if(map_.count({state, output}))
  {
    return {true, map_.at({state, output})};
  }
  return {false, {}};
}

void FSMController::TransitionMap::init(const FSMStateFactory & factory,
                                        const mc_rtc::Configuration & config)
{
  init_state_ = config("init", std::string{});
  auto transitions = config("transitions", std::vector<std::vector<std::string>>{});
  if(!transitions.size())
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "This FSM is not managed but its configuration does not hold a transitions entry or the transitions entry is empty.")
  }
  for(const auto & t : transitions)
  {
    if(t.size() < 3 || t.size() > 4)
    {
      LOG_ERROR_AND_THROW(std::runtime_error, "One of the transition entry is not valid")
    }
    auto str2type = [](const std::string & in)
    {
      if(in == "StepByStep")
      {
        return Transition::Type::StepByStep;
      }
      else if(in == "Auto")
      {
        return Transition::Type::Auto;
      }
      else if(in == "Strict")
      {
        return Transition::Type::Strict;
      }
      else
      {
        LOG_WARNING("Transition type (" << in << ") is not valid, defaulting to StepByStep")
        return Transition::Type::StepByStep;
      }
    };
    const auto & from = t[0];
    const auto & to = t[2];
    const auto & by = t[1];
    auto type = t.size() == 4 ? str2type(t[3]) : Transition::Type::StepByStep;
    if(!(factory.hasState(from) && factory.isValidOutput(from, by) && factory.hasState(to)))
    {
      LOG_ERROR("Invalid transition:")
      if(!factory.hasState(from))
      {
        LOG_ERROR("- origin state (" << from << ") is not loaded")
      }
      else if(!factory.isValidOutput(from, by))
      {
        LOG_ERROR("- origin state (" << from << ") has no output " << by)
      }
      if(!factory.hasState(to))
      {
        LOG_ERROR("- destination state (" << to << ") is not loaded")
      }
      continue;
    }
    if(map_.count({from, by}))
    {
      LOG_WARNING("Transition for (" << from << ", " << by << ") is specified more than once")
    }
    map_[{from, by}] = {to, type};
  }
}

const std::string & FSMController::TransitionMap::initState() const
{
  return init_state_;
}

std::ostream & FSMController::TransitionMap::print(std::ostream & os) const
{
  auto type2str = [](const FSMController::Transition::Type & t)
  {
    switch(t)
    {
#define MAKE_CASE(v)\
      case FSMController::Transition::Type::v: return #v;
      MAKE_CASE(StepByStep)
      MAKE_CASE(Auto)
      MAKE_CASE(Strict)
      default: return "Unknown transition type";
#undef MAKE_CASE
    };
  };
  for(const auto & t : map_)
  {
    os << t.first.first << " -- (" << t.first.second << ") --> " << t.second.state << " [" << type2str(t.second.type) << "]\n";
  }
  return os;
}

FSMController::FSMController(std::shared_ptr<mc_rbdyn::RobotModule> rm,
                             double dt,
                             const mc_rtc::Configuration & config)
: MCController(std::vector<mc_rbdyn::RobotModulePtr>{rm}, dt),
  config_(config),
  managed_(config("Managed", false)),
  step_by_step_(config("StepByStep", true)),
  factory_(config("StatesLibraries", std::vector<std::string>{}),
           config("StatesFiles", std::vector<std::string>{}),
           config("VerboseStateFactory", false))
{
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
        LOG_ERROR_AND_THROW(std::runtime_error, "FSM controller only handles robot modules that require two parameters at most")
      }
      if(!rm)
      {
        LOG_ERROR_AND_THROW(std::runtime_error, "Failed to load " << name << " as specified in configuration");
      }
      auto & r = robots().load(*rm);
      r.name(name);
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
      /*FIXME Add a name mechanism in ConstraintSet to get information here */
      solver().addConstraintSet(*constraints_.back());
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
      double weight = 1.0;
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
      double stiffness = 1.0;
      double weight = 1.0;
      if(config.has(robot.name()))
      {
        auto robot_config = config(robot.name());
        if(robot_config.has("com"))
        {
          robot_config("com")("stiffness", stiffness);
          robot_config("com")("weight", weight);
        }
      }
      auto t = std::make_shared<mc_tasks::CoMTask>(solver().robots(), robot.robotIndex(), stiffness, weight);
      t->name("FSM_" + t->name());
      com_tasks_[robot.name()] = t;
    }
  }
  /** Create contacts */
  contacts_ = config("contacts", std::set<FSMContact>{});
  contacts_changed_ = true;
  /** Load more states if they are provided in the configuration */
  if(config.has("states"))
  {
    factory_.load(config("states"));
  }
  /** Load transition map if necessary */
  if(!managed_)
  {
    transition_map_.init(factory_, config);
  }
}

bool FSMController::run()
{
  if(contacts_changed_)
  {
    std::vector<mc_rbdyn::Contact> contacts;
    for(const auto & c : contacts_)
    {
      contacts.emplace_back(robots(),
                            robots_idx_.at(c.r1),
                            robots_idx_.at(c.r2),
                            c.r1Surface,
                            c.r2Surface);
    }
    solver().setContacts(contacts);
    contacts_changed_ = false;
  }
  if(interrupt_triggered_)
  {
    interrupt_triggered_ = false;
    if(state_)
    {
      state_->stop(*this);
      state_->teardown(*this);
      LOG_WARNING("Interrupted " << curr_state_)
      state_ = nullptr;
      resetPostures();
    }
  }
  if(state_)
  {
    if(state_->run(*this))
    {
      LOG_SUCCESS("Completed " << curr_state_ << " (" << state_->output() << ")")
      resetPostures();
      state_output_ =  state_->output();
      state_->teardown(*this);
      if(managed_)
      {
        state_ = nullptr;
      }
      else
      {
        auto trans = transition_map_.transition(curr_state_, state_output_);
        if(trans.first)
        {
          next_state_ = trans.second.state;
          if(trans.second.type == Transition::Type::Auto ||
             (
              trans.second.type == Transition::Type::StepByStep &&
              !step_by_step_
             )
            )
          {
            nextState();
          }
          else
          {
            LOG_INFO("Waiting for user confirmation")
            state_ = nullptr;
          }
        }
        else
        {
          LOG_SUCCESS("FSM run completed")
          state_ = nullptr;
        }
      }
    }
  }
  else
  {
    if(transition_triggered_)
    {
      nextState();
    }
  }
  return MCController::run();
}

void FSMController::reset(const ControllerResetData & data)
{
  MCController::reset(data);
  resetPostures();
  if(!managed_)
  {
    next_state_ = transition_map_.initState();
    nextState();
  }
}

void FSMController::resetPostures()
{
  for(auto & pt : posture_tasks_)
  {
    pt.second->reset();
  }
  for(auto & ct : com_tasks_)
  {
    ct.second->reset();
    solver().addTask(ct.second);
  }
}

void FSMController::nextState()
{
  if(next_state_.empty()) { return; }
  LOG_INFO("Starting state " << next_state_)
  for(auto & ct : com_tasks_)
  {
    solver().removeTask(ct.second);
  }
  state_ = factory_.create(next_state_,
                           *this,
                           config_(next_state_, mc_rtc::Configuration{}));
  transition_triggered_ = false;
  curr_state_ = next_state_;
  next_state_ = "";
}

bool FSMController::play_next_stance()
{
  if(!managed_)
  {
    transition_triggered_ = true;
    return true;
  }
  return false;
}

bool FSMController::read_msg(std::string & msg)
{
  std::string token;
  std::stringstream ss;
  ss << msg;
  ss >> token;
  if(token == "interrupt")
  {
    interrupt_triggered_ = true;
    return true;
  }
  if(token == "play")
  {
    std::string state;
    ss >> state;
    if(!factory_.hasState(state))
    {
      LOG_ERROR("Cannot play unloaded state: " << state)
      return false;
    }
    interrupt_triggered_ = true;
    transition_triggered_ = true;
    next_state_ = state;
    return true;
  }
  return MCController::read_msg(msg);
}

bool FSMController::read_write_msg(std::string & msg,
                                   std::string & out)
{
  std::string token;
  std::stringstream ss;
  ss << msg;
  ss >> token;
  if(token == "current_state")
  {
    out = curr_state_;
    return true;
  }
  if(token == "output")
  {
    out = state_output_;
    return true;
  }
  if(token == "running")
  {
    out = state_ ? "1" : "0";
    return true;
  }
  return MCController::read_write_msg(msg, out);
}

void FSMController::addCollisions(const std::string & r1,
                                  const std::string & r2,
                                  const std::vector<mc_rbdyn::Collision> & collisions)
{
  if(!collision_constraints_.count({r1,r2}))
  {
    if(robots_idx_.count(r1) * robots_idx_.count(r2) == 0)
    {
      LOG_ERROR("Try to add collision for robot " << r1 << " and " << r2 << " which are not involved in this FSM")
      return;
    }
    collision_constraints_[{r1, r2}] = std::make_shared<mc_solver::CollisionsConstraint>(robots(), robots_idx_[r1], robots_idx_[r2], solver().dt());
    solver().addConstraintSet(*collision_constraints_[{r1, r2}]);
  }
  auto & cc = collision_constraints_[{r1, r2}];
  cc->addCollisions(solver(), collisions);
}

void FSMController::removeCollisions(const std::string & r1,
                                     const std::string & r2,
                                     const std::vector<mc_rbdyn::Collision> & collisions)
{
  if(!collision_constraints_.count({r1, r2}))
  {
    return;
  }
  auto & cc = collision_constraints_[{r1, r2}];
  cc->removeCollisions(solver(), collisions);
}

void FSMController::removeCollisions(const std::string & r1,
                                     const std::string & r2)
{
  if(!collision_constraints_.count({r1, r2}))
  {
    return;
  }
  auto & cc = collision_constraints_[{r1, r2}];
  cc->reset();
}

bool FSMController::hasRobot(const std::string & robot) const
{
  return robots_idx_.count(robot) != 0;
}

mc_rbdyn::Robot & FSMController::robot(const std::string & name)
{
  return solver().robot(robots_idx_.at(name));
}

std::shared_ptr<mc_tasks::PostureTask> FSMController::getPostureTask(const std::string & robot)
{
  if(posture_tasks_.count(robot))
  {
    return posture_tasks_.at(robot);
  }
  return nullptr;
}

void FSMController::addContact(const FSMContact & c)
{
  bool inserted;
  std::tie(std::ignore, inserted) = contacts_.insert(c);
  contacts_changed_ |= inserted;
}

void FSMController::removeContact(const FSMContact & c)
{
  contacts_changed_ |= contacts_.erase(c);
}

const std::set<FSMContact> & FSMController::contacts() const
{
  return contacts_;
}

bool FSMController::hasContact(const FSMContact & c) const
{
  for(const auto & co : contacts_)
  {
    if(co == c) { return true; }
  }
  return false;
}

}

CONTROLLER_CONSTRUCTOR("FSM", mc_control::FSMController)
