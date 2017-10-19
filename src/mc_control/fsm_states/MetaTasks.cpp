#include <mc_control/fsm_states/MetaTasks.h>

#include <mc_control/mc_fsm_controller.h>
#include <mc_tasks/MetaTaskLoader.h>

namespace mc_control
{

void MetaTasksState::configure(const mc_rtc::Configuration & config)
{
  auto entries = config("tasks", std::map<std::string, mc_rtc::Configuration>{});
  for(const auto & e : entries)
  {
    const auto & tName = e.first;
    const auto & tConfig = e.second;
    if(tasks_configs_.count(tName))
    {
      auto & currentConfig = tasks_configs_.at(tName);
      std::map<std::string, mc_rtc::Configuration> tEntries = tConfig;
      for(const auto & p : tEntries)
      {
        currentConfig.add(p.first, p.second);
      }
    }
    else
    {
      tasks_configs_[tName] = tConfig;
    }
  }
}

void MetaTasksState::start(FSMController & ctl)
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
      criterias_.emplace_back(tasks_.size() - 1,
                              [&ctl,&tConfig]()
                              {
                                CompletionCriteria crit;
                                crit.configure(ctl.solver().dt(), tConfig("completion"));
                                return crit;
                              }());
    }
  }
}

bool MetaTasksState::run(FSMController&)
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
    for(auto & c : criterias_)
    {
      auto & crit = c.second;
      const auto & t = *tasks_[c.first];
      LOG_INFO("Completed " << t.name() << " (" << crit.output() << ")")
    }
    output("OK");
    return true;
  }
  return false;
}

void MetaTasksState::teardown(FSMController & ctl)
{
  for(auto & t : tasks_)
  {
    ctl.solver().removeTask(t);
  }
}

}

EXPORT_SINGLE_STATE("MetaTasks", mc_control::MetaTasksState, "OK")
