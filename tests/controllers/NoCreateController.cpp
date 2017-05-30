/*! Only implements the MC_RTC_CONTROLLER function */
#include <mc_control/mc_controller.h>

extern "C"
{
  CONTROLLER_MODULE_API std::vector<std::string> MC_RTC_CONTROLLER() { return {"NoCreateController"}; }
  CONTROLLER_MODULE_API void destroy(mc_control::MCController * ctl)
  {
    delete ctl;
  }
}
