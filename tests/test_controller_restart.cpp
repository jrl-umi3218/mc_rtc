/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/mc_global_controller.h>

#include "utils.h"

#include <boost/filesystem.hpp>

static bool initialized = configureRobotLoader();

/** This tests checks that MCGlobalController can be deleted and constructed multiple time */

int main(int argc, char * argv[])
{
  if(argc < 3)
  {
    mc_rtc::log::critical("Wrong usage, expected: {} [conf] [nRuns]", argv[0]);
    return 1;
  }
  std::string config = argv[1];
  unsigned long nRuns = std::stoul(argv[2]);
  for(unsigned long i = 0; i < nRuns; ++i)
  {
    mc_control::MCGlobalController gc(config);
    for(size_t j = 0; j < 100; ++j)
    {
      gc.run();
    }
  }
}
