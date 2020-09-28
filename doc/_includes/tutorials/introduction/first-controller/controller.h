#pragma once

#include <mc_control/mc_controller.h>

#include "api.h"

struct MyFirstController_DLLAPI MyFirstController : public mc_control::MCController
{
    MyFirstController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

    bool run() override;

    void reset(const mc_control::ControllerResetData & reset_data) override;

    void switch_target();
private:
    mc_rtc::Configuration config_;
    std::string jointName = "NECK_Y";
    int jointIndex = 0;
    bool goingLeft = true;
};
