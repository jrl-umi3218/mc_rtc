/*! Only implements the MC_RTC_ROBOT_MODULE function */
#include <mc_rbdyn/RobotModule.h>

extern "C"
{
  ROBOT_MODULE_API std::vector<std::string> MC_RTC_ROBOT_MODULE() { return {"NoDestroyRobot"}; }
}
