#include <mc_control/mc_virtual_controller.h>

namespace mc_control
{

MCVirtualController::MCVirtualController()
: timeStep(0.002)
{
}

bool MCVirtualController::read_msg(std::string &)
{
  return false;
}

bool MCVirtualController::read_write_msg(std::string &, std::string &)
{
  return true;
}

}
