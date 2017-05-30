/*! This library voluntarily contains unresolved symbols */
#include <mc_rbdyn/RobotModule.h>

struct NotImplemented : public mc_rbdyn::RobotModule
{
  NotImplemented();
};

extern "C"
{
  ROBOT_MODULE_API std::vector<std::string> MC_RTC_ROBOT_MODULE() { return {"UnresolvedSymbolRobot"}; }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create()
  {
    return new NotImplemented();
  }
}
