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

  virtual bool read_msg(std::string & msg) override;
  virtual bool read_write_msg(std::string & msg, std::string & out) override;

  std::function<bool()> run_callback;
  std::function<void(const ControllerResetData &)> reset_callback;
  std::function<bool(std::string &)> read_msg_callback;
  std::function<PythonRWCallback(std::string &)> read_write_msg_callback;
};

} // namespace mc_control
