/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/states/MetaTasks.h>
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
      criterias_.emplace_back(tasks_.size() - 1, [&ctl, &tConfig, task]() {
        CompletionCriteria crit;
        crit.configure(*task, ctl.solver().dt(), tConfig("completion"));
        return crit;
      }());
    }
  }
}

bool MetaTasksState::run(Controller &)
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
}

} // namespace fsm

} // namespace mc_control

EXPORT_SINGLE_STATE("MetaTasks", mc_control::fsm::MetaTasksState)
