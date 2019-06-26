/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/logging.h>

#include "mc_bin_to_rosbag.h"
#include <sstream>

void usage(char * p)
{
  LOG_ERROR("Usage: " << p << " [bin] [bag] (dt=0.005)")
}

int main(int argc, char * argv[])
{
  if(argc < 3)
  {
    usage(argv[0]);
    return 1;
  }
  double dt = 0.005;
  if(argc > 3)
  {
    std::stringstream ss;
    ss << argv[3];
    ss >> dt;
  }
  mc_bin_to_rosbag(argv[1], argv[2], dt);
  return 0;
}
