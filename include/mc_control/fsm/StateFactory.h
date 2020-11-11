/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/fsm/State.h>
#include <mc_rtc/loader.h>

namespace mc_rtc
{
struct Configuration;
}

namespace mc_control
{

namespace fsm
{

struct MC_CONTROL_FSM_DLLAPI Controller;

/** \class StateFactory
 *
 * Creates State objects based on name and configuration entries.
 *
 * Also maintains a list of available states.
 *
 */
struct MC_CONTROL_FSM_DLLAPI StateFactory : public mc_rtc::ObjectLoader<State>
{
  /** Constructor
   *
   * Note: this class is based on mc_rtc::ObjectLoader but sandboxed
   * creation is systematically disabled
   *
   * \param paths List of paths to load states libraries from
   *
   * \param states List of states files (json/yaml) to read states from
   *
   * \bool verbose If true, output some warning information
   */
  StateFactory(const std::vector<std::string> & paths, const std::vector<std::string> & files, bool verbose);

  /** Load more states libraries */
  void load_libraries(const std::vector<std::string> & paths);

  /** Load more states files */
  void load_files(const std::vector<std::string> & files);

  /** Load from an mc_rtc::Configuration entry */
  void load(const std::map<std::string, mc_rtc::Configuration> & states);

  /** Load one state from a configuration entry
   *
   * Can be used to build a state programatically
   *
   * \param name Name of the new state provided by this configuration
   *
   * \param base Name of the state upon which this state should be based
   *
   * \param config New parameters' values
   *
   */
  void load(const std::string & name, const std::string & base, const mc_rtc::Configuration & config);

  /** Creates a state given its name, the owning controller and
   * configuration
   *
   * The state must have been loaded previously (either as a library or
   * through a state file. Otherwise the returned pointer is nullptr.
   *
   */
  StatePtr create(const std::string & state, Controller & ctl, const mc_rtc::Configuration & config);

  /** Creates a state without extra configuration */
  StatePtr create(const std::string & state, Controller & ctl);

  /** Query existence of a state */
  bool hasState(const std::string & state) const;

  /** Returns the list of loaded states */
  const std::vector<std::string> & states() const;

  /** Load state using a loader state, returns true if a state was loaded */
  bool load_with_loader(const std::string & state);

  /** Register a new loading function
   *
   * \param name Name that will be used to create new instance
   *
   * \param callback Callback that will be used to create the object
   *
   * \tparam RetT Must be derived from T
   *
   * \throws LoaderException if the object is already registered
   */
  template<typename RetT, typename... Args>
  void register_object(const std::string & name, std::function<RetT *(const Args &...)> callback)
  {
    mc_rtc::ObjectLoader<State>::register_object(name, callback);
    states_.push_back(name);
  }

private:
  /** Create a state from libraries or factory */
  StatePtr create(const std::string & state);
  /** Implementation for create */
  StatePtr create(const std::string & state,
                  Controller & ctl,
                  bool configure,
                  const mc_rtc::Configuration & config = {});
  /** Callback on state loading */
  void update(const std::string & cn);

private:
  std::vector<std::string> states_;
  using state_factory_fn = std::function<StatePtr(StateFactory &)>;
  std::map<std::string, state_factory_fn> states_factories_;
};

} // namespace fsm

} // namespace mc_control
