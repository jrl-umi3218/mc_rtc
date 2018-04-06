#include <mc_control/fsm/states/MetaTasks.h>

#include <mc_control/fsm/Controller.h>
#include <mc_tasks/MetaTaskLoader.h>

namespace mc_control
{

namespace fsm
{

void MetaTasksState::configure(const mc_rtc::Configuration & config)
{
  auto entries = config("tasks", std::map<std::string, mc_rtc::Configuration>{});
  for(const auto & e : entries)
  {
    const auto & tName = e.first;
    const auto & tConfig = e.second;
    tasks_configs_[tName].load(tConfig);
  }
  if(config.has("RemoveContacts"))
  {
    remove_contacts_config_ = config("RemoveContacts");
  }
  if(config.has("AddContacts"))
  {
    add_contacts_config_ = config("AddContacts");
  }
  if(config.has("RemoveContactsAfter"))
  {
    remove_contacts_after_config_ = config("RemoveContactsAfter");
  }
  if(config.has("AddContactsAfter"))
  {
    add_contacts_after_config_ = config("AddContactsAfter");
  }
  if(config.has("RemovePostureTask"))
  {
    remove_posture_task_ = config("RemovePostureTask");
  }
}

void MetaTasksState::start(Controller & ctl)
{
  for(auto & tc : tasks_configs_)
  {
    const auto & tName = tc.first;
    auto & tConfig = tc.second;
    if(!tConfig.has("name"))
    {
      tConfig.add("name", tName);
    }
    tasks_.push_back(mc_tasks::MetaTaskLoader::load(ctl.solver(), tConfig));
    ctl.solver().addTask(tasks_.back());
    if(tConfig.has("completion"))
    {
      std::pair<size_t, mc_control::CompletionCriteria> p = {tasks_.size() - 1, {}};
      auto task = tasks_.back();
      criterias_.emplace_back(tasks_.size() - 1,
                              [&ctl,&tConfig,task]()
                              {
                                CompletionCriteria crit;
                                crit.configure(*task, ctl.solver().dt(), tConfig("completion"));
                                return crit;
                              }());
    }
  }
  if(remove_contacts_config_.size())
  {
    std::set<Contact> removeContacts = remove_contacts_config_;
    for(const auto & c : removeContacts)
    {
      std::cout << "Remove contact " << c.r1Surface << "/" << c.r2Surface << std::endl;
      ctl.removeContact(c);
    }
  }
  if(add_contacts_config_.size())
  {
    std::set<Contact> addContacts = add_contacts_config_;
    for(const auto & c : addContacts)
    {
      std::cout << "Add contact " << c.r1Surface << "/" << c.r2Surface << " (dof: " << c.dof.transpose() << ")\n";
      ctl.addContact(c);
    }
  }
  if(remove_posture_task_)
  {
    ctl.solver().removeTask(ctl.getPostureTask(ctl.robot().name()));
  }
}

bool MetaTasksState::run(Controller&)
{
  bool finished = true;
  for(auto & c : criterias_)
  {
    auto & crit = c.second;
    const auto & t = *tasks_[c.first];
    finished = crit.completed(t) && finished;
  }
  if(finished)
  {
    if(!finished_first_)
    {
      finished_first_ = true;
      for(auto & c : criterias_)
      {
        auto & crit = c.second;
        const auto & t = *tasks_[c.first];
        LOG_INFO("Completed " << t.name() << " (" << crit.output() << ")")
      }
    }
    output("OK");
    return true;
  }
  return false;
}

void MetaTasksState::teardown(Controller & ctl)
{
  for(auto & t : tasks_)
  {
    ctl.solver().removeTask(t);
  }
  if(remove_posture_task_)
  {
    ctl.solver().addTask(ctl.getPostureTask(ctl.robot().name()));
  }
  if(remove_contacts_after_config_.size())
  {
    std::set<Contact> removeContacts = remove_contacts_after_config_;
    for(const auto & c : removeContacts)
    {
      std::cout << "Remove contact " << c.r1Surface << "/" << c.r2Surface << std::endl;
      ctl.removeContact(c);
    }
  }
  if(add_contacts_after_config_.size())
  {
    std::set<Contact> addContacts = add_contacts_after_config_;
    for(const auto & c : addContacts)
    {
      std::cout << "Add contact " << c.r1Surface << "/" << c.r2Surface << " (dof: " << c.dof.transpose() << ")\n";
      ctl.addContact(c);
    }
  }
}

} // namespace fsm

} // namespace mc_control

EXPORT_SINGLE_STATE("MetaTasks", mc_control::fsm::MetaTasksState, "OK")
