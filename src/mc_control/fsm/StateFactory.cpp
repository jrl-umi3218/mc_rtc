/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/fsm/StateFactory.h>
#include <mc_rtc/Configuration.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

namespace mc_control
{

namespace fsm
{

StateFactory::StateFactory(const std::vector<std::string> & paths, const std::vector<std::string> & files, bool verbose)
: mc_rtc::ObjectLoader<State>("MC_RTC_FSM_STATE", {}, verbose)
{
  load_libraries(paths);
  load_files(files);
}

void StateFactory::load_libraries(const std::vector<std::string> & paths)
{
  mc_rtc::ObjectLoader<State>::load_libraries(paths,
                                              [this](const std::string & cn, mc_rtc::LTDLHandle &) { update(cn); });
}

namespace
{

// The code in this namespace allows to deal with scattered states
// information. For example if State B depends on State A but we attempt to
// load State B before State A. It also detects cycle (State B depends on
// State A, State A depends on State B).

/** Undefined state */
struct UDState
{
  std::string state;
  std::string base;
  mc_rtc::Configuration config;
};

/** One pass of resolution
 *
 * For each undefined state, if the base exists we create the state and
 * remove the undefined state from the states' vector
 *
 */
void resolve_pass(StateFactory & factory, std::vector<UDState> & states)
{
  for(auto it = states.begin(); it != states.end();)
  {
    auto & uds = *it;
    if(factory.hasState(uds.base) || factory.load_with_loader(uds.base))
    {
      factory.load(uds.state, uds.base, uds.config);
      it = states.erase(it);
    }
    else
    {
      ++it;
    }
  }
}

/** Full resolution
 *
 * Do passes until a pass does nothing or all states have been created
 *
 */
void resolve(StateFactory & factory, std::vector<UDState> & states)
{
  size_t prev_size = 0;
  while(states.size() != 0)
  {
    resolve_pass(factory, states);
    if(states.size() == prev_size)
    {
      break;
    }
    prev_size = states.size();
  }
  if(states.size() != 0)
  {
    mc_rtc::log::error("Some states could not be loaded as their base is not available, check for typos or cycles");
    for(const auto & s : states)
    {
      mc_rtc::log::warning("- {} (base: {})", s.state, s.base);
    }
  }
}

/** Build up a list of undefined states */
void load_ud(StateFactory & factory,
             const std::map<std::string, mc_rtc::Configuration> & states,
             std::vector<UDState> & ud_states)
{
  for(const auto & s : states)
  {
    const auto & config = s.second;
    std::string base = config("base", std::string(""));
    if(base.empty())
    {
      mc_rtc::log::error("Attempted to load state {} but no base is specified in the configuration", s.first);
      continue;
    }
    if(factory.hasState(base) || factory.load_with_loader(base))
    {
      factory.load(s.first, base, config);
    }
    else
    {
      ud_states.push_back({s.first, base, config});
    }
  }
}

void load_file(StateFactory & factory, const std::string & file, std::vector<UDState> & ud_states, bool verbose)
{
  if(verbose)
  {
    mc_rtc::log::info("Load {}", file);
  }
  std::map<std::string, mc_rtc::Configuration> states = mc_rtc::Configuration(file);
  load_ud(factory, states, ud_states);
}

void load_dir(StateFactory & factory, const std::string & dir, std::vector<UDState> & ud_states, bool verbose)
{
  bfs::directory_iterator dit(dir), endit;
  std::vector<bfs::path> drange;
  std::copy(dit, endit, std::back_inserter(drange));
  for(const auto & p : drange)
  {
    if(bfs::is_regular_file(p))
    {
      load_file(factory, p.string(), ud_states, verbose);
    }
    else if(bfs::is_directory(p))
    {
      load_dir(factory, p.string(), ud_states, verbose);
    }
  }
}

} // namespace

void StateFactory::load_files(const std::vector<std::string> & files)
{
  std::vector<UDState> ud_states;
  for(const auto & f : files)
  {
    if(bfs::is_directory(f))
    {
      mc_rtc::log::info("Looking for state files in {}", f);
      load_dir(*this, f, ud_states, verbose);
    }
    else
    {
      if(bfs::exists(f) && bfs::is_regular_file(f))
      {
        load_file(*this, f, ud_states, verbose);
      }
      else
      {
        mc_rtc::log::warning("State file {} does not exist", f);
      }
    }
  }
  if(ud_states.size())
  {
    resolve(*this, ud_states);
  }
}

void StateFactory::load(const std::map<std::string, mc_rtc::Configuration> & states)
{
  std::vector<UDState> ud_states;
  load_ud(*this, states, ud_states);
  if(ud_states.size())
  {
    resolve(*this, ud_states);
  }
}

void StateFactory::load(const std::string & name, const std::string & base, const mc_rtc::Configuration & config)
{
  if(!hasState(base) && !load_with_loader(base))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Cannot create a state using a base {} that does not exist", base);
  }
  if(hasState(name))
  {
#ifndef MC_RTC_BUILD_STATIC
    mc_rtc::log::error_and_throw<std::runtime_error>("State {} already exists", name);
#else
    states_.erase(std::find(states_.begin(), states_.end(), name));
#endif
  }
  if(verbose)
  {
    mc_rtc::log::info("New state from file: {} (base: {})", name, base);
  }
  states_.push_back(name);
  states_configurations_[name] = {base, "", config};
}

StatePtr StateFactory::create(const std::string & state, Controller & ctl, const mc_rtc::Configuration & config)
{
  return create(state, ctl, true, config);
}

StatePtr StateFactory::create(const std::string & state, Controller & ctl)
{
  return create(state, ctl, false);
}

StatePtr StateFactory::create(const std::string & state,
                              Controller & ctl,
                              bool configure,
                              const mc_rtc::Configuration & config)
{
  StatePtr ret = create(state, state);
  if(!ret)
  {
    mc_rtc::log::error("Creation of {} state failed", state);
    return nullptr;
  }
  if(configure)
  {
    ret->configure_(config);
  }
  ret->start_(ctl);
  return ret;
}

StatePtr StateFactory::create(const std::string & state, const mc_rtc::Configuration & config)
{
  StatePtr ret = create(state);
  ret->configure_(config);
  return ret;
}

StatePtr StateFactory::create(const std::string & state)
{
  return create(state, state);
}

StatePtr StateFactory::create(const std::string & state, const std::string & final_name)
{
  StatePtr ret = nullptr;
  if(!hasState(state))
  {
    mc_rtc::log::error("Attempted to create unavailable state {}", state);
    return nullptr;
  }
  if(has_object(state))
  {
    ret = create_object(state);
    ret->name(final_name);
  }
  else if(states_configurations_.count(state))
  {
    const auto & config = states_configurations_[state];
    if(config.arg.size())
    {
      ret = create_object(config.base, config.arg);
      ret->name(final_name);
    }
    else
    {
      ret = create(config.base, final_name);
    }
    ret->configure_(config.config);
  }
  if(!ret)
  {
    mc_rtc::log::error("Creation of {} state failed", state);
    return nullptr;
  }
  return ret;
}

bool StateFactory::hasState(const std::string & state) const
{
  return std::find(states_.begin(), states_.end(), state) != states_.end();
}

bool StateFactory::load_with_loader(const std::string & state)
{
  auto sharp = state.find('#');
  if(sharp == std::string::npos || sharp == state.size() - 1)
  {
    return false;
  }
  std::string loader = state.substr(0, sharp);
  std::string arg = state.substr(sharp + 1);
  if(!has_object(loader))
  {
    mc_rtc::log::error("Cannot create state {}, loader {} has not been loaded", state, loader);
    return false;
  }
  if(verbose)
  {
    mc_rtc::log::info("New state: {} provided by loader: {}", state, loader);
  }
  states_.push_back(state);
  states_configurations_[state] = {loader, arg, {}};
  return true;
}

const std::vector<std::string> & StateFactory::states() const
{
  return states_;
}

void StateFactory::update(const std::string & cn)
{
  if(verbose)
  {
    mc_rtc::log::info("New state from library: {}", cn);
  }
  states_.push_back(cn);
}

} // namespace fsm

} // namespace mc_control
