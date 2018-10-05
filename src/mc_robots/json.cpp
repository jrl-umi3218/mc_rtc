#include "json.h"

#include <mc_rbdyn/configuration_io.h>

extern "C"
{

  mc_rbdyn::RobotModule * create(const std::string &, const std::string & path)
  {
    mc_rtc::Configuration config(path);
    mc_rbdyn::RobotModule rm(config);
    return new mc_rbdyn::RobotModule(rm);
  }
}
