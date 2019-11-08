#pragma once

#include <mc_control/mc_controller.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/SurfaceTransformTask.h>


#include "api.h"

enum DoorPhase
{
      APPROACH = 0,
      HANDLE,
      OPEN
};

struct MyFirstController_DLLAPI MyFirstController : public mc_control::MCController
{
    MyFirstController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

    bool run() override;

    void reset(const mc_control::ControllerResetData & reset_data) override;

    void switch_phase();
private:
    mc_rtc::Configuration config_;
    std::shared_ptr<mc_tasks::CoMTask> comTask;
    std::shared_ptr<mc_solver::KinematicsConstraint> doorKinematics;
    std::shared_ptr<mc_tasks::PostureTask> doorPosture;
    std::shared_ptr<mc_tasks::SurfaceTransformTask> handTask;
    DoorPhase phase = APPROACH;
};
