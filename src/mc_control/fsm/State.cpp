/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/fsm/State.h>

#include <mc_control/fsm/Controller.h>

#include <mc_solver/ConstraintSetLoader.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_rbdyn/configuration_io.h>

namespace
{

/** Reads a list of distance limits from a config entry, preferring the "distanceLimits" key and falling back on the
 * deprecated "collisions" key */
std::vector<mc_rbdyn::DistanceLimit> distanceLimitsFromConfig(const mc_rtc::Configuration & c)
{
  if(c.has("distanceLimits")) { return c("distanceLimits"); }
  if(c.has("collisions"))
  {
    mc_rtc::log::deprecated("State", "collisions", "distanceLimits");
    return c("collisions");
  }
  return {};
}

} // namespace

namespace mc_control
{

namespace fsm
{

void State::configure_(const mc_rtc::Configuration & config)
{
  if(config.has("RemoveContacts")) { remove_contacts_config_.load(config("RemoveContacts")); }
  if(config.has("AddContacts")) { add_contacts_config_.load(config("AddContacts")); }
  if(config.has("RemoveContactsAfter")) { remove_contacts_after_config_.load(config("RemoveContactsAfter")); }
  if(config.has("AddContactsAfter")) { add_contacts_after_config_.load(config("AddContactsAfter")); }
  if(config.has("RemoveDistanceLimits")) { remove_distance_limits_config_.load(config("RemoveDistanceLimits")); }
  else if(config.has("RemoveCollisions"))
  {
    mc_rtc::log::deprecated("State", "RemoveCollisions", "RemoveDistanceLimits");
    remove_distance_limits_config_.load(config("RemoveCollisions"));
  }
  if(config.has("AddDistanceLimits")) { add_distance_limits_config_.load(config("AddDistanceLimits")); }
  else if(config.has("AddCollisions"))
  {
    mc_rtc::log::deprecated("State", "AddCollisions", "AddDistanceLimits");
    add_distance_limits_config_.load(config("AddCollisions"));
  }
  if(config.has("RemoveDistanceLimitsAfter"))
  {
    remove_distance_limits_after_config_.load(config("RemoveDistanceLimitsAfter"));
  }
  else if(config.has("RemoveCollisionsAfter"))
  {
    mc_rtc::log::deprecated("State", "RemoveCollisionsAfter", "RemoveDistanceLimitsAfter");
    remove_distance_limits_after_config_.load(config("RemoveCollisionsAfter"));
  }
  if(config.has("AddDistanceLimitsAfter")) { add_distance_limits_after_config_.load(config("AddDistanceLimitsAfter")); }
  else if(config.has("AddCollisionsAfter"))
  {
    mc_rtc::log::deprecated("State", "AddCollisionsAfter", "AddDistanceLimitsAfter");
    add_distance_limits_after_config_.load(config("AddCollisionsAfter"));
  }
  if(config.has("constraints")) { constraints_config_.load(config("constraints")); }
  if(config.has("tasks")) { tasks_config_.load(config("tasks")); }
  if(config.has("RemovePostureTask"))
  {
    mc_rtc::log::warning("[MC_RTC_DEPRECATED][{}] RemovePostureTask is deprecated, use DisablePostureTask instead",
                         name());
    remove_posture_task_.load(config("RemovePostureTask"));
  }
  if(config.has("DisablePostureTask")) { remove_posture_task_.load(config("DisablePostureTask")); }
  configure(config);
}

void State::configure(const mc_rtc::Configuration & config)
{
  config_.load(config);
}

void State::start_(Controller & ctl)
{
  if(remove_contacts_config_.size())
  {
    ContactSet removeContacts = remove_contacts_config_;
    for(const auto & c : removeContacts) { ctl.removeContact(c); }
  }
  if(add_contacts_config_.size())
  {
    ContactSet addContacts = add_contacts_config_;
    for(const auto & c : addContacts) { ctl.addContact(c); }
  }
  if(remove_distance_limits_config_.size())
  {
    for(const auto & c : remove_distance_limits_config_)
    {
      std::string r1 = c("r1");
      std::string r2 = r1;
      if(c.has("r2")) { r2 = static_cast<std::string>(c("r2")); }
      if(c.has("distanceLimits") || c.has("collisions"))
      {
        ctl.removeDistanceLimits(r1, r2, distanceLimitsFromConfig(c));
      }
      else
      {
        ctl.removeDistanceLimits(r1, r2);
      }
    }
  }
  if(add_distance_limits_config_.size())
  {
    for(const auto & c : add_distance_limits_config_)
    {
      std::string r1 = c("r1");
      std::string r2 = r1;
      if(c.has("r2")) { r2 = static_cast<std::string>(c("r2")); }
      ctl.addDistanceLimits(r1, r2, distanceLimitsFromConfig(c));
    }
  }
  if(!remove_posture_task_.empty())
  {
    if(!remove_posture_task_.size())
    {
      bool remove = remove_posture_task_;
      if(remove)
      {
        for(const auto & robot : ctl.robots())
        {
          auto pt = ctl.getPostureTask(robot.name());
          if(pt)
          {
            ctl.solver().removeTask(pt);
            postures_.push_back(pt);
          }
        }
      }
    }
    else
    {
      std::vector<std::string> robots = remove_posture_task_;
      for(const auto & k : robots)
      {
        auto pt = ctl.getPostureTask(k);
        if(pt)
        {
          ctl.solver().removeTask(pt);
          postures_.push_back(pt);
        }
      }
    }
  }
  if(!constraints_config_.empty())
  {
    std::map<std::string, mc_rtc::Configuration> constraints = constraints_config_;
    for(const auto & c : constraints)
    {
      constraints_.push_back(mc_solver::ConstraintSetLoader::load(ctl.solver(), c.second));
      ctl.solver().addConstraintSet(*constraints_.back());
    }
  }
  if(!tasks_config_.empty())
  {
    std::map<std::string, mc_rtc::Configuration> tasks = tasks_config_;
    for(auto & t : tasks)
    {
      const auto & tName = t.first;
      auto & tConfig = t.second;
      if(!tConfig.has("name")) { tConfig.add("name", tName); }
      tasks_.push_back({mc_tasks::MetaTaskLoader::load(ctl.solver(), tConfig), tConfig});
      ctl.solver().addTask(tasks_.back().first);
    }
  }
  start(ctl);
}

void State::teardown_(Controller & ctl)
{
  for(const auto & pt : postures_) { ctl.solver().addTask(pt); }
  if(remove_contacts_after_config_.size())
  {
    ContactSet removeContacts = remove_contacts_after_config_;
    for(const auto & c : removeContacts) { ctl.removeContact(c); }
  }
  if(add_contacts_after_config_.size())
  {
    ContactSet addContacts = add_contacts_after_config_;
    for(const auto & c : addContacts) { ctl.addContact(c); }
  }
  if(remove_distance_limits_after_config_.size())
  {
    for(const auto & c : remove_distance_limits_after_config_)
    {
      std::string r1 = c("r1");
      std::string r2 = r1;
      if(c.has("r2")) { r2 = static_cast<std::string>(c("r2")); }
      if(c.has("distanceLimits") || c.has("collisions"))
      {
        ctl.removeDistanceLimits(r1, r2, distanceLimitsFromConfig(c));
      }
      else
      {
        ctl.removeDistanceLimits(r1, r2);
      }
    }
  }
  if(add_distance_limits_after_config_.size())
  {
    for(const auto & c : add_distance_limits_after_config_)
    {
      std::string r1 = c("r1");
      std::string r2 = r1;
      if(c.has("r2")) { r2 = static_cast<std::string>(c("r2")); }
      ctl.addDistanceLimits(r1, r2, distanceLimitsFromConfig(c));
    }
  }
  for(const auto & c : constraints_) { ctl.solver().removeConstraintSet(*c); }
  for(const auto & t : tasks_) { ctl.solver().removeTask(t.first); }
  teardown(ctl);
}

} // namespace fsm

} // namespace mc_control
