/*! Only implements the CLASS_NAME function */
#include <mc_rbdyn/RobotModule.h>

extern "C"
{
  ROBOT_MODULE_API const char * CLASS_NAME() { return "NoDestroyRobot"; }
}
