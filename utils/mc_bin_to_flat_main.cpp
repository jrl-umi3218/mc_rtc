/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/logging.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#include "mc_bin_to_flat.h"

void usage(const char * bin)
{
  mc_rtc::log::error("Usage: {} [bin] ([flat])", bin);
}

int main(int argc, char * argv[])
{
  if(argc != 3 && argc != 2)
  {
    usage(argv[0]);
    return 1;
  }
  std::string in = argv[1];
  std::string out = "";
  if(argc == 3)
  {
    out = argv[2];
  }
  else
  {
    out = bfs::path(argv[1]).filename().replace_extension(".flat").string();
    if(out == in)
    {
      mc_rtc::log::error("Please specify a different output name");
      return 1;
    }
    mc_rtc::log::info("Output converted log to {}", out);
  }
  mc_bin_to_flat(in, out);
  return 0;
}
