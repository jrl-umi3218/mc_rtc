/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/logging.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#include "mc_bin_to_log.h"

void usage(const char * bin)
{
  LOG_ERROR("Usage: " << bin << " [bin] ([log])")
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
    out = bfs::path(argv[1]).filename().replace_extension(".csv").string();
    if(out == in)
    {
      LOG_ERROR("Please specify a different output name")
      return 1;
    }
    LOG_INFO("Output converted log to " << out)
  }
  mc_bin_to_log(in, out);
  return 0;
}
