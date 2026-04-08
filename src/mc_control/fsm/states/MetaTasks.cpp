/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/states/MetaTasks.h>
#include <mc_rtc/ConfigurationHelpers.h>
#include <mc_rtc/io_utils.h>
#include <mc_tasks/MetaTaskLoader.h>

namespace mc_control
{

namespace fsm
{

void MetaTasksState::configure(const mc_rtc::Configuration & config)
{
  outputCrit_ = mc_rtc::fromVectorOrElement<std::string>(config, "outputs", outputCrit_);
}

void MetaTasksState::start(Controller & ctl)
{
  for(size_t i = 0; i < tasks_.size(); ++i)
  {
    const auto & t = tasks_[i].first;
    const auto & tConfig = tasks_[i].second;
    if(tConfig.has("completion"))
    {
      CompletionCriteria crit;
      crit.configure(*t, ctl.solver().dt(), tConfig("completion"));
      auto & tCrit = criterias_[t->name()];
      tCrit.idx = i;
      tCrit.criteria = crit;
      tCrit.use_output = false;
    }
  }
  // Check validity of tasks output names
  for(const auto & tName : outputCrit_)
  {
    if(!tasks_config_.has(tName))
    {
      mc_rtc::log::error_and_throw(
          "[{}] Invalid output task name {}: should be one of the following tasks: {}. Check your \"outputs\" "
          "configuration.",
          name(), tName, mc_rtc::io::to_string(tasks_config_.keys()));
    }
    else
    {
      const std::string & name = tasks_config_(tName)("name", tName);
      criterias_[name].use_output = true;
    }
  }
}

bool MetaTasksState::run(Controller &)
{
  bool finished = true;
  for(auto & c : criterias_)
  {
    auto & tCrit = c.second;
    const auto & t = *tasks_[tCrit.idx].first;
    finished = tCrit.criteria.completed(t) && finished;
  }
  if(finished)
  {
    if(!finished_first_)
    {
      finished_first_ = true;
      std::string out = "";
      for(auto & c : criterias_)
      {
        auto & tCrit = c.second;
        auto & crit = tCrit.criteria;
        const auto & t = *tasks_[tCrit.idx].first;
        if(tCrit.use_output)
        {
          if(out.size())
          {
            out += ", ";
          }
          out += t.name() + "=" + crit.output();
        }
        mc_rtc::log::info("Completed {} ({})", t.name(), crit.output());
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

void MetaTasksState::teardown(Controller &) {}

} // namespace fsm

} // namespace mc_control

EXPORT_SINGLE_STATE("MetaTasks", mc_control::fsm::MetaTasksState)
