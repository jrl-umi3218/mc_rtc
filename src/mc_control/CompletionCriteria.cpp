/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/CompletionCriteria.h>

namespace mc_control
{

bool CompletionCriteria::completed(const mc_tasks::MetaTask & task)
{
  output_ = "";
  return fn_(task, output_);
}

void CompletionCriteria::configure(const mc_tasks::MetaTask & task, double dt, const mc_rtc::Configuration & config)
{
  fn_ = build(task, dt, config);
}

const std::string & CompletionCriteria::output() const
{
  return output_;
}

std::function<bool(const mc_tasks::MetaTask &, std::string &)> CompletionCriteria::build(
    const mc_tasks::MetaTask & task,
    double dt,
    const mc_rtc::Configuration & config)
{
  if(config.has("timeout"))
  {
    double tick = 0;
    double goal = config("timeout");
    assert(goal > 0);
    goal = std::ceil(goal / dt);
    return [tick, goal](const mc_tasks::MetaTask &, std::string & out) mutable {
      if(++tick > goal)
      {
        out += "timeout";
        return true;
      }
      return false;
    };
  }
  if(config.has("eval"))
  {
    double norm = config("eval");
    assert(norm > 0);
    return [norm](const mc_tasks::MetaTask & t, std::string & out) {
      if(t.eval().norm() < norm)
      {
        out += "eval";
        return true;
      }
      return false;
    };
  }
  if(config.has("speed"))
  {
    double norm = config("speed");
    assert(norm > 0);
    return [norm](const mc_tasks::MetaTask & t, std::string & out) {
      if(t.speed().norm() < norm)
      {
        out += "speed";
        return true;
      }
      return false;
    };
  }
  if(config.has("OR"))
  {
    std::array<mc_rtc::Configuration, 2> conds = config("OR");
    auto lhs = build(task, dt, conds[0]);
    auto rhs = build(task, dt, conds[1]);
    return [lhs, rhs](const mc_tasks::MetaTask & t, std::string & out) {
      if(lhs(t, out))
      {
        return true;
      }
      else if(rhs(t, out))
      {
        return true;
      }
      return false;
    };
  }
  if(config.has("AND"))
  {
    std::array<mc_rtc::Configuration, 2> conds = config("AND");
    auto lhs = build(task, dt, conds[0]);
    auto rhs = build(task, dt, conds[1]);
    return [lhs, rhs](const mc_tasks::MetaTask & t, std::string & out) {
      if(lhs(t, out))
      {
        out += " AND ";
        return rhs(t, out);
      }
      return false;
    };
  }
  return task.buildCompletionCriteria(dt, config);
}

} // namespace mc_control
