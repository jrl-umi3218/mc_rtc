#include <mc_control/GlobalPluginLoader.h>

namespace mc_control
{

std::unique_ptr<mc_rtc::ObjectLoader<GlobalPlugin>> GlobalPluginLoader::loader_;

mc_rtc::ObjectLoader<GlobalPlugin> & GlobalPluginLoader::loader()
{
  if(loader_)
  {
    return *loader_;
  }
  loader_.reset(new mc_rtc::ObjectLoader<GlobalPlugin>("MC_RTC_GLOBAL_PLUGIN", {}, false, false));
  return *loader_;
}

} // namespace mc_control
