#include <mc_control/mc_completion_criteria.h>

namespace mc_control
{

bool CompletionCriteria::completed(const mc_tasks::MetaTask & task)
{
  output_ = "";
  return fn_(task, output_);
}

void CompletionCriteria::configure(double dt, const mc_rtc::Configuration & config)
{
  fn_ = build(dt, config);
}

const std::string & CompletionCriteria::output() const
{
  return output_;
}

std::function<bool(const mc_tasks::MetaTask & task, std::string&)> CompletionCriteria::build(double dt, const mc_rtc::Configuration & config)
{
  if(config.has("timeout"))
  {
    double tick = 0;
    double goal = config("timeout");
    assert(goal > 0);
    goal = std::ceil(goal/dt);
    return [tick, goal](const mc_tasks::MetaTask&, std::string & out) mutable
    {
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
    return [norm](const mc_tasks::MetaTask & task, std::string & out)
    {
      if(task.eval().norm() < norm)
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
    return [norm](const mc_tasks::MetaTask & task, std::string & out)
    {
      if(task.speed().norm() < norm)
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
    auto lhs = build(dt, conds[0]);
    auto rhs = build(dt, conds[1]);
    return [lhs, rhs](const mc_tasks::MetaTask & task, std::string & out)
    {
      if(lhs(task, out))
      {
        return true;
      }
      else if(rhs(task, out))
      {
        return true;
      }
      return false;
    };
  }
  if(config.has("AND"))
  {
    std::array<mc_rtc::Configuration, 2> conds = config("AND");
    auto lhs = build(dt, conds[0]);
    auto rhs = build(dt, conds[1]);
    return [lhs, rhs](const mc_tasks::MetaTask & task, std::string & out)
    {
      if(lhs(task, out))
      {
        out += " AND ";
        return rhs(task, out);
      }
      return false;
    };
  }
  return [](const mc_tasks::MetaTask&, std::string&) { return true; };
}

}
