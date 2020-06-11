/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_text_controller.h"

#include <mc_rbdyn/configuration_io.h>
#include <mc_rtc/logging.h>
#include <mc_solver/ConstraintSetLoader.h>
#include <mc_tasks/MetaTaskLoader.h>

namespace mc_control
{

MCTextController::MCTextController(std::shared_ptr<mc_rbdyn::RobotModule> robot,
                                   double dt,
                                   const mc_rtc::Configuration & config)
: MCController(robot, dt)
{
  if(!config.has("Text"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "No entries relative to the Text controller in the loaded configuration");
  }
  config_ = config("Text");
}

void MCTextController::reset(const mc_control::ControllerResetData & data)
{
  mc_control::MCController::reset(data);
  if(!config_.has("constraints"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("No constraints in the provided text file");
  }
  if(!config_.has("tasks"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("No tasks in the provided text file");
  }
  for(const auto & c : config_("constraints"))
  {
    constraints_.push_back(mc_solver::ConstraintSetLoader::load(solver(), c));
    solver().addConstraintSet(*constraints_.back());
  }
  solver().addTask(postureTask);
  for(const auto & t : config_("tasks"))
  {
    tasks_.push_back(mc_tasks::MetaTaskLoader::load(solver(), t));
    solver().addTask(tasks_.back());
  }
  std::vector<mc_rbdyn::Contact> contacts = {};
  if(config_.has("contacts"))
  {
    contacts = mc_rbdyn::Contact::loadVector(robots(), config_("contacts"));
  }
  solver().setContacts(contacts);
}

} // namespace mc_control
