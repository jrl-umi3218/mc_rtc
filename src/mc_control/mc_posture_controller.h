#pragma once

#include <mc_control/mc_controller.h>

#include <mc_control/api.h>

namespace mc_control
{

struct MC_CONTROL_DLLAPI MCPostureController : public MCController
{
public:
  /* Common stuff */
  MCPostureController(std::shared_ptr<mc_rbdyn::RobotModule> robot, double dt);
};

}

SIMPLE_CONTROLLER_CONSTRUCTOR("Posture", mc_control::MCPostureController)
