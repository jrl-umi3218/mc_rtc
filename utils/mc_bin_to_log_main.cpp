/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/logging.h>

#include "mc_bin_to_log.h"

void usage(const char * bin)
{
  LOG_ERROR("Usage: " << bin << " [bin] [log]")
}

int main(int argc, char * argv[])
{
  if(argc != 3)
  {
    usage(argv[0]);
    return 1;
  }
  mc_bin_to_log(argv[1], argv[2]);
  return 0;
}
