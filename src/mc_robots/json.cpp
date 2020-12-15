/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "json.h"

#include <mc_rbdyn/configuration_io.h>

#ifndef MC_RTC_BUILD_STATIC

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    names = {"json"};
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string &, const std::string & path)
  {
    mc_rtc::Configuration config(path);
    mc_rbdyn::RobotModule rm(config);
    return new mc_rbdyn::RobotModule(rm);
  }
}

#else

#  include <mc_rbdyn/RobotLoader.h>

namespace
{

static auto registered = []() {
  using fn_t = std::function<mc_rbdyn::RobotModule *(const std::string &)>;
  fn_t callback = [](const std::string & path) {
    mc_rtc::Configuration config(path);
    mc_rbdyn::RobotModule rm(config);
    return new mc_rbdyn::RobotModule(rm);
  };
  mc_rbdyn::RobotLoader::register_object("json", callback);
  return true;
}();
}

#endif
