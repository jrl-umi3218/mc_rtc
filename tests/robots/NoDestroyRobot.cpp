/*! Only implements the MC_RTC_ROBOT_MODULE function */
#include <mc_rbdyn/RobotModule.h>

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    names = {"NoDestroyRobot"};
  }
}
