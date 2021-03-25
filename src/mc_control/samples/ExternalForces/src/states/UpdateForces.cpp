/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "UpdateForces.h"
#include <mc_control/fsm/Controller.h>
#include <mc_rtc/ConfigurationHelpers.h>
#include "../mc_external_forces_controller.h"

void UpdateForces::configure(const mc_rtc::Configuration & config)
{
  config_.load(config);
}

void UpdateForces::start(mc_control::fsm::Controller & _ctl)
{
  auto & ctl = static_cast<ExternalForcesController &>(_ctl);

  for(auto task : ctl.solver().tasks())
  {
    if(auto stabilizerTask = dynamic_cast<mc_tasks::lipm_stabilizer::StabilizerTask *>(task))
    {
      stabilizerTask_ = stabilizerTask;
    }
    else if(auto impedanceTask = dynamic_cast<mc_tasks::force::ImpedanceTask *>(task))
    {
      impedanceTasks_.push_back(impedanceTask);
    }
  }

  startTime_ = ctl.t();

  output("OK");
}

bool UpdateForces::run(mc_control::fsm::Controller & _ctl)
{
  auto & ctl = static_cast<ExternalForcesController &>(_ctl);

  // Set target external forces of StabilizerTask and ImpedanceTask
  sva::ForceVecd targetWrench(Eigen::Vector3d::Zero(),
                              Eigen::Vector3d(0, 0, -20 * (1 - std::cos(0.5 * (ctl.t() - startTime_)))));
  std::vector<std::string> surfaceNames;
  std::vector<sva::ForceVecd> targetWrenches;
  std::vector<sva::MotionVecd> gains;
  for(const auto & impedanceTask : impedanceTasks_)
  {
    impedanceTask->targetWrench(targetWrench);
    surfaceNames.push_back(impedanceTask->surface());
    targetWrenches.push_back(targetWrench);
    gains.emplace_back(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ());
  }
  stabilizerTask_->setExternalWrenches(surfaceNames, targetWrenches, gains);

  return false;
}

void UpdateForces::teardown(mc_control::fsm::Controller &) {}

EXPORT_SINGLE_STATE("ExternalForces::UpdateForces", UpdateForces)
