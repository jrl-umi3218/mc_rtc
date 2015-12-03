#include <mc_control/mc_virtual_controller.h>

namespace mc_control
{

MCVirtualController::MCVirtualController(double dt)
: timeStep(dt)
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
