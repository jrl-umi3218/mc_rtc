/*
 * Copyright 2015-2023 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/GlobalPlugin.h>

#include <mc_rtc/log/FlatLog.h>

namespace mc_plugin
{

struct Replay : public mc_control::GlobalPlugin
{
  inline GlobalPluginConfiguration configuration() override
  {
    GlobalPluginConfiguration out;
    out.should_always_run = false;
    out.should_run_before = true;
    out.should_run_after = true;
    return out;
  }

  /** The Replay plugin configuration supports the same options as those in Ticker::Configuration::Replay
   *
   * The exceptions are:
   * - stop_after_log and exit_after_log are not supported
   * - if Replay::Log is set in the datastore before this plugin is started then it is assumed the log was loaded
   *   previously
   */
  void init(mc_control::MCGlobalController & gc, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & gc) override;

  void before(mc_control::MCGlobalController & gc) override;

  void after(mc_control::MCGlobalController & gc) override;

  using update_datastore_fn_t = void (*)(const mc_rtc::log::FlatLog & log,
                                         const std::string & log_entry,
                                         size_t idx,
                                         mc_rtc::DataStore & ds,
                                         const std::string & ds_entry);

private:
  std::string ctl_name_;
  size_t iters_ = 0;
  std::shared_ptr<mc_rtc::log::FlatLog> log_;
  bool pause_ = false;
  bool with_inputs_ = true;
  bool with_gui_inputs_ = true;
  bool with_outputs_ = false;
  std::map<std::string, std::string> log_to_datastore_;
  struct DataStoreUpdate
  {
    std::string log_entry;
    std::string ds_entry;
    update_datastore_fn_t update;
  };
  std::vector<DataStoreUpdate> datastore_updates_;
  std::shared_ptr<mc_rbdyn::Robots> robots_;
};

} // namespace mc_plugin
