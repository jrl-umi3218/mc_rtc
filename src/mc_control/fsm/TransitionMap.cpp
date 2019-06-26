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
  return {false, {}};
}

std::vector<std::string> TransitionMap::transitions(const std::string & state) const
{
  std::vector<std::string> ret;
  for(const auto & t : map_)
  {
    if(t.first.first == state)
    {
      ret.push_back(t.second.state);
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
    LOG_ERROR_AND_THROW(std::runtime_error, "Cannot load TransitionMap if the configuration does not hold a "
                                            "transitions entry or the transitions entry is empty.")
  }
  bool valid_init_state = false;
  std::string first_valid_state = "";
  for(const auto & t : transitions)
  {
    if(t.size() < 3 || t.size() > 4)
    {
      LOG_ERROR_AND_THROW(std::runtime_error, "One of the transition entry is not valid")
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
        LOG_WARNING("Transition type (" << in << ") is not valid, defaulting to StepByStep")
        return Transition::Type::StepByStep;
      }
    };
    const auto & from = t[0];
    const auto & to = t[2];
    const auto & by = t[1];
    auto type = t.size() == 4 ? str2type(t[3]) : Transition::Type::StepByStep;
    if(!(factory.hasState(from) && factory.hasState(to)))
    {
      LOG_ERROR("Invalid transition:")
      if(!factory.hasState(from))
      {
        LOG_ERROR("- origin state (" << from << ") is not loaded")
      }
      if(!factory.hasState(to))
      {
        LOG_ERROR("- destination state (" << to << ") is not loaded")
      }
      continue;
    }
    if(map_.count({from, by}))
    {
      LOG_WARNING("Transition for (" << from << ", " << by << ") is specified more than once")
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
    LOG_ERROR_AND_THROW(std::runtime_error,
                        "None of the transitions you attempted to load in this TransitionMap are valid.")
  }
  if(!valid_init_state)
  {
    if(init_state_.size())
    {
      LOG_WARNING("The initial state you provided ("
                  << init_state_
                  << ") is not valid, will replace it with the first valid state in the transitions map: "
                  << first_valid_state)
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
