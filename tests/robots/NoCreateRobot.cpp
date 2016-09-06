/*! Only implements the CLASS_NAME/destroy functions */
#include <mc_rbdyn/RobotModule.h>

extern "C"
{
  ROBOT_MODULE_API const char * CLASS_NAME() { return "NoCreateRobot"; }
  void destroy(mc_rbdyn::RobotModule * mod)
  {
    delete mod;
  }
}
