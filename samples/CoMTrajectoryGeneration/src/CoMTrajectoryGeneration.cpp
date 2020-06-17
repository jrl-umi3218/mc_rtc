#include "CoMTrajectoryGeneration.h"

CoMTrajectoryGeneration::CoMTrajectoryGeneration(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{

  mc_rtc::log::success("CoMTrajectoryGeneration init done ");
}

bool CoMTrajectoryGeneration::run()
{
  return mc_control::fsm::Controller::run();
}

void CoMTrajectoryGeneration::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}


