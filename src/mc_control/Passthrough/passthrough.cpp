/*
 * Copyright 2015-2023 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/mc_controller.h>

namespace mc_control
{

struct MC_CONTROL_DLLAPI Passthrough : public MCController
{
  Passthrough(mc_rbdyn::RobotModulePtr robot, double dt) : MCController(robot, dt) {}

  void reset(const ControllerResetData &) override {}

  bool run() override
  {
    return true;
  }
};

} // namespace mc_control

SIMPLE_CONTROLLER_CONSTRUCTOR("Passthrough", mc_control::Passthrough)
