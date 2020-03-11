/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/states/MetaTasks.h>
#include <mc_rtc/ConfigurationHelpers.h>
#include <mc_rtc/io_utils.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <algorithm>

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

  if(config.has("outputs"))
  {
    outputCrit_ = mc_rtc::fromVectorOrElement<std::string>(config, "outputs");
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
  // Check validity of tasks output names
  for(const auto & tName : outputCrit_)
  {
    if(!tasks_configs_.count(tName))
    {
      LOG_ERROR_AND_THROW(
          std::runtime_error,
          "[" << name() << "] Invalid output task name " << tName << ": should be one of the following tasks: "
              << mc_rtc::io::to_string(tasks_, [](const mc_tasks::MetaTaskPtr & t) { return t->name(); })
              << ". Check your \"outputs\" configuration.");
    }
  }
}

bool MetaTasksState::run(Controller &)
{
  bool finished = true;
  std::string out = "";
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
        if(std::find(outputCrit_.begin(), outputCrit_.end(), t.name()) != std::end(outputCrit_))
        {
          if(out.size())
          {
            out += ", ";
          }
          out += t.name() + "=" + crit.output();
        }
        LOG_INFO("Completed " << t.name() << " (" << crit.output() << ")")
      }
      if(out.empty())
      {
        out = "OK";
      }
      output(out);
    }
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
