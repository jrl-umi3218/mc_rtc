/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

/*! This library create function throws */
#include <mc_rbdyn/RobotModule.h>
#include <mc_rtc/config.h>

struct FPERobot : public mc_rbdyn::RobotModule
{
  FPERobot() : RobotModule(std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH), std::string("ground"))
  {
    int a = 0;
    b = 42 / a;
  }
  int b;
};

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    names = {"FPERobot"};
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create()
  {
    return new FPERobot();
  }
}
