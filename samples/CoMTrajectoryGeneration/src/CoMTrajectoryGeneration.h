#pragma once

#include <mc_control/mc_controller.h>
#include <mc_control/fsm/Controller.h>

#include "api.h"

struct CoMTrajectoryGeneration_DLLAPI CoMTrajectoryGeneration : public mc_control::fsm::Controller
{
    CoMTrajectoryGeneration(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

    bool run() override;

    void reset(const mc_control::ControllerResetData & reset_data) override;
private:
    mc_rtc::Configuration config_;
};