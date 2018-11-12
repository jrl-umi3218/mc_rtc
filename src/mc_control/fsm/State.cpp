#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>

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
  config("RemovePostureTask", remove_posture_task_);
  configure(config);
}

void State::start_(Controller & ctl)
{
  if(remove_contacts_config_.size())
  {
    ContactSet removeContacts = remove_contacts_config_;
    for(const auto & c : removeContacts)
    {
      std::cout << "Remove contact " << c.r1Surface << "/" << c.r2Surface << std::endl;
      ctl.removeContact(c);
    }
  }
  if(add_contacts_config_.size())
  {
    ContactSet addContacts = add_contacts_config_;
    for(const auto & c : addContacts)
    {
      std::cout << "Add contact " << c.r1Surface << "/" << c.r2Surface << " (dof: " << c.dof.transpose() << ")\n";
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
  if(remove_posture_task_)
  {
    ctl.solver().removeTask(ctl.getPostureTask(ctl.robot().name()));
  }
  start(ctl);
}

void State::teardown_(Controller & ctl)
{
  if(remove_posture_task_)
  {
    ctl.solver().addTask(ctl.getPostureTask(ctl.robot().name()));
  }
  if(remove_contacts_after_config_.size())
  {
    ContactSet removeContacts = remove_contacts_after_config_;
    for(const auto & c : removeContacts)
    {
      std::cout << "Remove contact " << c.r1Surface << "/" << c.r2Surface << std::endl;
      ctl.removeContact(c);
    }
  }
  if(add_contacts_after_config_.size())
  {
    ContactSet addContacts = add_contacts_after_config_;
    for(const auto & c : addContacts)
    {
      std::cout << "Add contact " << c.r1Surface << "/" << c.r2Surface << " (dof: " << c.dof.transpose() << ")\n";
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
  teardown(ctl);
}

} // namespace fsm

} // namespace mc_control
