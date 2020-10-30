#include <mc_control/ControllerLoader.h>

namespace mc_control
{

std::unique_ptr<mc_rtc::ObjectLoader<MCController>> ControllerLoader::loader_;

mc_rtc::ObjectLoader<MCController> & ControllerLoader::loader()
{
  if(loader_)
  {
    return *loader_;
  }
  loader_.reset(new mc_rtc::ObjectLoader<MCController>("MC_RTC_CONTROLLER", {}, false, false));
  return *loader_;
}

}
