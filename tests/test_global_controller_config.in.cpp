#include <string>

std::string get_config_file()
{
  return "@CONFIGURATION_FILE@";
}

unsigned int nrIter()
{
  // clang-format off
  return @RUN_ITER@;
  // clang-format on
}

std::string next_controller()
{
  return "@NEXT_CONTROLLER@";
}
