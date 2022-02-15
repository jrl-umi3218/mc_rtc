/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/fsm/TransitionMap.h>

namespace mc_control
{

namespace fsm
{

std::pair<bool, Transition> TransitionMap::transition(const std::string & state, const std::string & output) const
{
  if(map_.count({state, output}))
  {
    return {true, map_.at({state, output})};
  }
  else if(map_.count({state, "DEFAULT"}))
  {
    return {true, map_.at({state, "DEFAULT"})};
  }
  return {false, {}};
}

std::unordered_set<std::string> TransitionMap::transitions(const std::string & state) const
{
  std::unordered_set<std::string> ret;
  for(const auto & t : map_)
  {
    if(t.first.first == state)
    {
      ret.insert(t.second.state);
    }
  }
  return ret;
}

void TransitionMap::init(const StateFactory & factory, const mc_rtc::Configuration & config)
{
  init_state_ = config("init", std::string{});
  auto transitions = config("transitions", std::vector<std::vector<std::string>>{});
  if(!transitions.size())
  {
    mc_rtc::log::error_and_throw("Cannot load TransitionMap if the configuration does not hold a "
                                 "transitions entry or the transitions entry is empty.");
  }
  bool valid_init_state = false;
  std::string first_valid_state = "";
  for(const auto & t : transitions)
  {
    if(t.size() < 3 || t.size() > 4)
    {
      mc_rtc::log::error_and_throw("One of the transition entry is not valid");
    }
    auto str2type = [](const std::string & in) {
      if(in == "StepByStep")
      {
        return Transition::Type::StepByStep;
      }
      else if(in == "Auto")
      {
        return Transition::Type::Auto;
      }
      else if(in == "Strict")
      {
        return Transition::Type::Strict;
      }
      else
      {
        mc_rtc::log::warning("Transition type ({}) is not valid, defaulting to StepByStep", in);
        return Transition::Type::StepByStep;
      }
    };
    const auto & from = t[0];
    const auto & to = t[2];
    const auto & by = t[1];
    auto type = t.size() == 4 ? str2type(t[3]) : Transition::Type::StepByStep;
    if(!(factory.hasState(from) && factory.hasState(to)))
    {
      mc_rtc::log::error("Invalid transition:");
      if(!factory.hasState(from))
      {
        mc_rtc::log::error("- origin state ({}) is not loaded", from);
      }
      if(!factory.hasState(to))
      {
        mc_rtc::log::error("- destination state ({}) is not loaded", to);
      }
      continue;
    }
    if(map_.count({from, by}))
    {
      mc_rtc::log::warning("Transition for ({}, {}) is specified more than once", from, by);
    }
    if(init_state_ == from)
    {
      valid_init_state = true;
    }
    if(first_valid_state.size() == 0)
    {
      first_valid_state = from;
    }
    map_[{from, by}] = {to, type};
  }
  if(map_.size() == 0)
  {
    mc_rtc::log::error_and_throw("None of the transitions you attempted to load in this TransitionMap are valid.");
  }
  if(!valid_init_state)
  {
    if(init_state_.size())
    {
      mc_rtc::log::warning(
          "The initial state you provided ({}) is not valid, will replace it with the first valid state in the "
          "transitions map: {}",
          init_state_, first_valid_state);
    }
    init_state_ = first_valid_state;
  }
}

const std::string & TransitionMap::initState() const
{
  return init_state_;
}

std::ostream & TransitionMap::print(std::ostream & os) const
{
  auto type2str = [](const Transition::Type & t) {
    switch(t)
    {
#define MAKE_CASE(v)        \
  case Transition::Type::v: \
    return #v;
      MAKE_CASE(StepByStep)
      MAKE_CASE(Auto)
      MAKE_CASE(Strict)
      default:
        return "Unknown transition type";
#undef MAKE_CASE
    };
  };
  for(const auto & t : map_)
  {
    os << t.first.first << " -- (" << t.first.second << ") --> " << t.second.state << " [" << type2str(t.second.type)
       << "]\n";
  }
  return os;
}

} // namespace fsm

} // namespace mc_control
