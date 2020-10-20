/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/fsm/State.h>

#include <mc_control/fsm/Controller.h>

#include <mc_solver/ConstraintSetLoader.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_rbdyn/configuration_io.h>

namespace mc_control
{

namespace fsm
{

void State::configure_(const mc_rtc::Configuration & config)
{
  if(config.has("RemoveContacts"))
  {
    remove_contacts_config_.load(config("RemoveContacts"));
  }
  if(config.has("AddContacts"))
  {
    add_contacts_config_.load(config("AddContacts"));
  }
  if(config.has("RemoveContactsAfter"))
  {
    remove_contacts_after_config_.load(config("RemoveContactsAfter"));
  }
  if(config.has("AddContactsAfter"))
  {
    add_contacts_after_config_.load(config("AddContactsAfter"));
  }
  if(config.has("RemoveCollisions"))
  {
    remove_collisions_config_.load(config("RemoveCollisions"));
  }
  if(config.has("AddCollisions"))
  {
    add_collisions_config_.load(config("AddCollisions"));
  }
  if(config.has("RemoveCollisionsAfter"))
  {
    remove_collisions_after_config_.load(config("RemoveCollisionsAfter"));
  }
  if(config.has("AddCollisionsAfter"))
  {
    add_collisions_after_config_.load(config("AddCollisionsAfter"));
  }
  if(config.has("constraints"))
  {
    constraints_config_.load(config("constraints"));
  }
  if(config.has("tasks"))
  {
    tasks_config_.load(config("tasks"));
  }
  if(config.has("RemovePostureTask"))
  {
    mc_rtc::log::warning("[MC_RTC_DEPRECATED][{}] RemovePostureTask is deprecated, use DisablePostureTask instead",
                         name());
    remove_posture_task_.load(config("RemovePostureTask"));
  }
  if(config.has("DisablePostureTask"))
  {
    remove_posture_task_.load(config("DisablePostureTask"));
  }
  configure(config);
}

void State::start_(Controller & ctl)
{
  if(remove_contacts_config_.size())
  {
    ContactSet removeContacts = remove_contacts_config_;
    for(const auto & c : removeContacts)
    {
      ctl.removeContact(c);
    }
  }
  if(add_contacts_config_.size())
  {
    ContactSet addContacts = add_contacts_config_;
    for(const auto & c : addContacts)
    {
      ctl.addContact(c);
    }
  }
  if(remove_collisions_config_.size())
  {
    for(const auto & c : remove_collisions_config_)
    {
      std::string r1 = c("r1");
      std::string r2 = r1;
      if(c.has("r2"))
      {
        r2 = static_cast<std::string>(c("r2"));
      }
      if(c.has("collisions"))
      {
        std::vector<mc_rbdyn::Collision> collisions = c("collisions");
        ctl.removeCollisions(r1, r2, collisions);
      }
      else
      {
        ctl.removeCollisions(r1, r2);
      }
    }
  }
  if(add_collisions_config_.size())
  {
    for(const auto & c : add_collisions_config_)
    {
      std::string r1 = c("r1");
      std::string r2 = r1;
      if(c.has("r2"))
      {
        r2 = static_cast<std::string>(c("r2"));
      }
      std::vector<mc_rbdyn::Collision> collisions = c("collisions");
      ctl.addCollisions(r1, r2, collisions);
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
      if(!tConfig.has("name"))
      {
        tConfig.add("name", tName);
      }
      tasks_.push_back({mc_tasks::MetaTaskLoader::load(ctl.solver(), tConfig), tConfig});
      ctl.solver().addTask(tasks_.back().first);
    }
  }
  start(ctl);
}

void State::teardown_(Controller & ctl)
{
  for(const auto & pt : postures_)
  {
    ctl.solver().addTask(pt);
  }
  if(remove_contacts_after_config_.size())
  {
    ContactSet removeContacts = remove_contacts_after_config_;
    for(const auto & c : removeContacts)
    {
      ctl.removeContact(c);
    }
  }
  if(add_contacts_after_config_.size())
  {
    ContactSet addContacts = add_contacts_after_config_;
    for(const auto & c : addContacts)
    {
      ctl.addContact(c);
    }
  }
  if(remove_collisions_after_config_.size())
  {
    for(const auto & c : remove_collisions_after_config_)
    {
      std::string r1 = c("r1");
      std::string r2 = r1;
      if(c.has("r2"))
      {
        r2 = static_cast<std::string>(c("r2"));
      }
      if(c.has("collisions"))
      {
        std::vector<mc_rbdyn::Collision> collisions = c("collisions");
        ctl.removeCollisions(r1, r2, collisions);
      }
      else
      {
        ctl.removeCollisions(r1, r2);
      }
    }
  }
  if(add_collisions_after_config_.size())
  {
    for(const auto & c : add_collisions_after_config_)
    {
      std::string r1 = c("r1");
      std::string r2 = r1;
      if(c.has("r2"))
      {
        r2 = static_cast<std::string>(c("r2"));
      }
      std::vector<mc_rbdyn::Collision> collisions = c("collisions");
      ctl.addCollisions(r1, r2, collisions);
    }
  }
  for(const auto & c : constraints_)
  {
    ctl.solver().removeConstraintSet(*c);
  }
  for(const auto & t : tasks_)
  {
    ctl.solver().removeTask(t.first);
  }
  teardown(ctl);
}

} // namespace fsm

} // namespace mc_control
