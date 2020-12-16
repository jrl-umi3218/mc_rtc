#include "PreviewControlWalking.h"

PreviewControlWalking::PreviewControlWalking(mc_rbdyn::RobotModulePtr rm,
                                             double dt,
                                             const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
  datastore().make_call("KinematicAnchorFrame::" + robot().name(), [this](const mc_rbdyn::Robot & robot) {
    return sva::interpolate(robot.surfacePose("RightFootCenter"), robot.surfacePose("LeftFootCenter"), 0.5);
  });

  mc_rtc::log::success("PreviewControlWalking init done ");
}

bool PreviewControlWalking::run()
{
  return mc_control::fsm::Controller::run();
}

void PreviewControlWalking::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);

  dt_ = solver().dt();
}
