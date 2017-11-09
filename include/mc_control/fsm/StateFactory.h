#pragma once

#include <mc_rtc/loader.h>

#include <mc_control/fsm/State.h>

namespace mc_rtc
{
  struct Configuration;
}

namespace mc_control
{

namespace fsm
{

struct MC_CONTROL_DLLAPI Controller;

/** \class StateFactory
 *
 * Creates State objects based on name and configuration entries.
 *
 * Also maintains a list of available states and outputs provided by said
 * states.
 *
 */
struct MC_CONTROL_DLLAPI StateFactory : public mc_rtc::ObjectLoader<State>
{
  /** Constructor
   *
   * Note: this class is based on mc_rtc::ObjectLoader but sandboxed
   * creation is systematically disabled
   *
   * \param paths List of paths to load states libraries from
   *
   * \param states List of states files (json) to read states from
   *
   * \bool verbose If true, output some warning information
   */
  StateFactory(const std::vector<std::string> & paths,
               const std::vector<std::string> & files,
               bool verbose);

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
  void load(const std::string & name,
            const std::string & base,
            const mc_rtc::Configuration & config);

  /** Creates a state given its name, the owning controller and
   * configuration
   *
   * The state must have been loaded previously (either as a library or
   * through a state file. Otherwise the returned pointer is nullptr.
   *
   */
  StatePtr create(const std::string & state,
                  Controller & ctl,
                  const mc_rtc::Configuration & config);

  /** Query existence of a state */
  bool hasState(const std::string & state) const;

  /** Returns the list of loaded states */
  const std::vector<std::string> & states() const;

  /** Returns the list of outputs for a given state
   *
   * If the state does not exist, an empty list is returned
   *
   * Throws if the state does not exist.
   */
  const std::vector<std::string> & stateOutputs(const std::string & state) const;

  /** Returns true if the provided output is part of the provided state's
   * outputs */
  bool isValidOutput(const std::string & state,
                     const std::string & output) const;
private:
  /** Create a state from libraries or factory */
  StatePtr create(const std::string & state);
  /** Update the outputs list */
  void update(const std::string & cn, lt_dlhandle handle);
private:
  std::vector<std::string> states_;
  std::map<std::string, std::vector<std::string>> outputs_;
  using state_factory_fn = std::function<StatePtr(StateFactory&)>;
  std::map<std::string, state_factory_fn> states_factories_;
};

} // namespace fsm

} // namespace mc_control
