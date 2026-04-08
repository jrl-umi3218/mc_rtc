/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

/*! Only implements the MC_RTC_CONTROLLER function */
#include <mc_control/mc_controller.h>

extern "C"
{
  CONTROLLER_MODULE_API void MC_RTC_CONTROLLER(std::vector<std::string> & names)
  {
    names = {"NoCreateController"};
  }
  CONTROLLER_MODULE_API void destroy(mc_control::MCController * ctl)
  {
    delete ctl;
  }
}
