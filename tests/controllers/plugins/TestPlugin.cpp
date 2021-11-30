/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/GlobalPluginMacros.h>

namespace mc_plugin
{

struct MC_CONTROL_DLLAPI TestPlugin : public mc_control::GlobalPlugin
{
  TestPlugin(const std::string & name) : name_(name) {}

  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override
  {
    config_.load(config);
    reset(controller);
  }

  void reset(mc_control::MCGlobalController & controller) override
  {
    iter_before = 0;
    iter_after = 0;
    auto & ds = controller.controller().datastore();
    ds.make<size_t>(name_ + "::iter_before", iter_before);
    ds.make<size_t>(name_ + "::iter_after", iter_after);
    ds.make<void *>(name_ + "::address", this);
    ds.make<mc_rtc::Configuration>(name_ + "::config", config_);
  }

  void before(mc_control::MCGlobalController & controller) override
  {
    iter_before++;
    auto & ds = controller.controller().datastore();
    ds.assign(name_ + "::iter_before", iter_before);
  }

  void after(mc_control::MCGlobalController & controller) override
  {
    iter_after++;
    auto & ds = controller.controller().datastore();
    ds.assign(name_ + "::iter_after", iter_after);
  }

private:
  mc_rtc::Configuration config_;
  std::string name_;
  size_t iter_before = 0;
  size_t iter_after = 0;
};

} // namespace mc_plugin

extern "C"
{

  GLOBAL_PLUGIN_API void MC_RTC_GLOBAL_PLUGIN(std::vector<std::string> & names)
  {
    names = {"Plugin0", "Plugin1", "Plugin2", "Plugin3"};
  }

  GLOBAL_PLUGIN_API void destroy(mc_control::GlobalPlugin * ptr)
  {
    delete ptr;
  }

  GLOBAL_PLUGIN_API mc_control::GlobalPlugin * create(const std::string & name)
  {
    return new mc_plugin::TestPlugin(name);
  }
}
