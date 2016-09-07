/*! Only implements the CLASS_NAME function */
#include <mc_control/mc_controller.h>

extern "C"
{
  CONTROLLER_MODULE_API const char * CLASS_NAME() { return "NoCreateController"; }
}
