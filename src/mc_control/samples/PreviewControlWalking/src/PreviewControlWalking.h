#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/mc_controller.h>

#include "api.h"

struct PreviewControlWalking_DLLAPI PreviewControlWalking : public mc_control::fsm::Controller
{
  PreviewControlWalking(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  double t()
  {
    return logger().t();
  }

  double dt_ = 0;

private:
  mc_rtc::Configuration config_;
};
