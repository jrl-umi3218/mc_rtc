/*! Only implements the MC_RTC_ROBOT_MODULE/destroy functions */
#include <mc_rbdyn/RobotModule.h>

extern "C"
{
  ROBOT_MODULE_API std::vector<std::string> MC_RTC_ROBOT_MODULE() { return {"NoCreateRobot"}; }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * mod)
  {
    delete mod;
  }
}
