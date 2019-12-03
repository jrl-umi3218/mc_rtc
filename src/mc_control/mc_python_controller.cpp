/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/mc_python_controller.h>

namespace mc_control
{

MCPythonController::MCPythonController(const std::vector<std::shared_ptr<mc_rbdyn::RobotModule>> & robots, double dt)
: MCController(robots, dt)
{
}

void MCPythonController::reset(const ControllerResetData & reset_data)
{
  MCController::reset(reset_data);
  if(reset_callback)
  {
    reset_callback(reset_data);
  }
}

bool MCPythonController::run()
{
  bool ret = MCController::run();
  if(ret)
  {
    if(run_callback)
    {
      ret = ret && run_callback();
    }
  }
  return ret;
}

} // namespace mc_control
