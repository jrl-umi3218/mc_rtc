/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/logging.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#include "mc_bin_to_rosbag.h"
#include <sstream>

void usage(char * p)
{
  mc_rtc::log::error("Usage: {} [bin] ([bag]) (dt=0.005)", p);
}

int main(int argc, char * argv[])
{
  if(argc < 2)
  {
    usage(argv[0]);
    return 1;
  }
  std::string in = argv[1];
  std::string out = "";
  if(argc > 2)
  {
    out = argv[2];
  }
  else
  {
    out = bfs::path(argv[1]).filename().replace_extension(".bag").string();
    if(out == in)
    {
      mc_rtc::log::error("Please specify a different output name");
      return 1;
    }
    mc_rtc::log::info("Output converted log to {}", out);
  }
  double dt = 0.005;
  if(argc > 3)
  {
    std::stringstream ss;
    ss << argv[3];
    ss >> dt;
  }
  mc_bin_to_rosbag(in, out, dt);
  return 0;
}
