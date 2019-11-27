/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/api.h>
#include <mc_control/mc_controller.h>

namespace mc_control
{

struct PythonRWCallback
{
  bool success;
  std::string out;
};

struct MC_CONTROL_DLLAPI MCPythonController : public MCController
{
public:
  MCPythonController(const std::vector<std::shared_ptr<mc_rbdyn::RobotModule>> & robots, double dt);

  virtual void reset(const ControllerResetData & reset_data) override;

  virtual bool run() override;

  std::function<bool()> run_callback;
  std::function<void(const ControllerResetData &)> reset_callback;
};

} // namespace mc_control
