/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

/*! This library create function throws */
#include <mc_rbdyn/RobotModule.h>
#include <mc_rtc/config.h>
#include <mc_rtc/pragma.h>

struct SegfaultRobot : public mc_rbdyn::RobotModule
{
  SegfaultRobot() : RobotModule(std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH), std::string("ground"))
  {
    MC_RTC_diagnostic_push
    MC_RTC_diagnostic_ignored(GCC, "-Wnull-dereference")
    *(int *)0 = 0;
    MC_RTC_diagnostic_pop
  }
};

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    names = {"SegfaultRobot"};
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create()
  {
    return new SegfaultRobot();
  }
}
