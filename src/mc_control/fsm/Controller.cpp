/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/fsm/Controller.h>

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/configuration_io.h>

#include <mc_rtc/gui/Button.h>
#include <mc_rtc/gui/Form.h>
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/io_utils.h>

namespace mc_control
{

namespace fsm
{

#ifdef MC_RTC_BUILD_STATIC
std::unique_ptr<StateFactory> Controller::factory_ptr_;
#endif

Controller::Controller(std::shared_ptr<mc_rbdyn::RobotModule> rm, double dt, const mc_rtc::Configuration & config)
: MCController(std::vector<mc_rbdyn::RobotModulePtr>{rm}, dt, config),
#ifndef MC_RTC_BUILD_STATIC
  factory_(config("StatesLibraries", std::vector<std::string>{}),
           config("StatesFiles", std::vector<std::string>{}),
           config("VerboseStateFactory", false))
#else
  factory_(factory())
#endif
{
#ifdef MC_RTC_BUILD_STATIC
  factory_.load_files(config("StatesFiles", std::vector<std::string>{}));
  factory_.set_verbosity(config("VerboseStateFactory", false));
#endif
  idle_keep_state_ = config("IdleKeepState", false);
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
  /** Load more states if they are provided in the configuration */
  if(config.has("states"))
  {
    factory_.load(config("states"));
  }
  /** Setup executor */
  executor_.init(*this, config_);
}

Controller::~Controller()
{
  executor_.teardown(*this);
  datastore().clear();
}

bool Controller::run()
{
  return run(mc_solver::FeedbackType::None);
}

bool Controller::run(mc_solver::FeedbackType fType)
{
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
  MCController::reset(data);
  /** GUI information */
  if(gui_)
  {
    auto all_states = factory_.states();
    std::sort(all_states.begin(), all_states.end());
    gui_->data().add("states", all_states);
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
  // Save posture weights
  saved_posture_weights_.clear();
  for(auto & pt : posture_tasks_)
  {
    saved_posture_weights_[pt.first] = pt.second->weight();
    // Set high weight to prevent the robot from changing configuration
    pt.second->weight(10000);
  }
  for(auto & fft : ff_tasks_)
  {
    fft.second->reset();
    solver().addTask(fft.second);
  }
}

void Controller::teardownIdleState()
{
  // Reset default posture weights
  for(auto & pt : posture_tasks_)
  {
    pt.second->weight(saved_posture_weights_[pt.first]);
  }

  for(auto & fft : ff_tasks_)
  {
    solver().removeTask(fft.second);
  }
}

std::shared_ptr<mc_tasks::PostureTask> Controller::getPostureTask(const std::string & robot)
{
  if(posture_tasks_.count(robot))
  {
    return posture_tasks_.at(robot);
  }
  return nullptr;
}

bool Controller::resume(const std::string & state)
{
  if(!factory_.hasState(state))
  {
    mc_rtc::log::error("Cannot play unloaded state: {}", state);
    return false;
  }
  return executor_.resume(state);
}

} // namespace fsm

} // namespace mc_control
