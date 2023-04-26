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
    python_failed_ = handle_python_error();
  }
}

bool MCPythonController::run()
{
  if(python_failed_) { return false; }
  bool ret = MCController::run();
  if(ret && run_callback)
  {
    ret = ret && run_callback();
    python_failed_ = handle_python_error();
  }
  return ret && !python_failed_;
}

} // namespace mc_control
