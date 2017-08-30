#pragma once

#include <mc_control/mc_controller.h>
#include <mc_tasks/MetaTask.h>

#include <mc_control/api.h>

/** This controllers shows how a controller can be entirely configured from text files */

namespace mc_control
{

struct MC_CONTROL_DLLAPI MCTextController : public MCController
{
public:
  MCTextController(std::shared_ptr<mc_rbdyn::RobotModule> robot, double dt, const mc_rtc::Configuration & config);

  void reset(const mc_control::ControllerResetData & data) override;
private:
  mc_rtc::Configuration config_;
  std::vector<mc_solver::ConstraintSetPtr> constraints_;
  std::vector<mc_tasks::MetaTaskPtr> tasks_;
};

}

CONTROLLER_CONSTRUCTOR("Text", mc_control::MCTextController)
