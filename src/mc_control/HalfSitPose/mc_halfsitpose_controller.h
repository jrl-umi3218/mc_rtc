/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/api.h>
#include <mc_control/mc_controller.h>

#include <vector>

namespace mc_control
{

struct MC_CONTROL_DLLAPI MCHalfSitPoseController : public MCController
{
public:
  /* Common stuff */
  MCHalfSitPoseController(std::shared_ptr<mc_rbdyn::RobotModule> robot, double dt);

  virtual void reset(const ControllerResetData &) override;

private:
  std::vector<std::vector<double>> halfSitPose;
};

} // namespace mc_control
