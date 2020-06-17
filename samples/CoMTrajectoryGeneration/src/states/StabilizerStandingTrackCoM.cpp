#include "StabilizerStandingTrackCoM.h"
#include "../CoMTrajectoryGeneration.h"
#include <mc_rtc/io_utils.h>

#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/lipm_stabilizer/StabilizerTask.h>

namespace mc_samples
{

void StabilizerStandingTrackCoM::configure(const mc_rtc::Configuration & config)
{
  config_.load(config);
}

void StabilizerStandingTrackCoM::start(mc_control::fsm::Controller & ctl)
{
  // create stabilizer task from config
  if(!config_.has("StabilizerConfig"))
  {
    config_.add("StabilizerConfig");
  }
  config_("StabilizerConfig").add("type", "lipm_stabilizer");

  stabilizerTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::lipm_stabilizer::StabilizerTask>(
      ctl.solver(), config_("StabilizerConfig"));
  // FIXME seems to hang when solution is not feasible
  ctl.solver().addTask(stabilizerTask_);
  stabilizerTask_->staticTarget(ctl.realRobot().com());

  ctl.datastore().make_call("StabilizerStandingTrackCoM::target",
                            [this](const Eigen::Vector3d & com, const Eigen::Vector3d & comd, const Eigen::Vector3d & comdd, const Eigen::Vector3d & zmp)
                            {
                              stabilizerTask_->target(com, comd, comdd, zmp);
                            });
}


bool StabilizerStandingTrackCoM::run(mc_control::fsm::Controller & ctl)
{
  // Update anchor frame for the KinematicInertial observer
  ctl.anchorFrame(stabilizerTask_->anchorFrame());
  ctl.anchorFrameReal(stabilizerTask_->anchorFrameReal());
  output("OK");
  return false;
}

void StabilizerStandingTrackCoM::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.datastore().remove("StabilizerStandingTrackCoM::target");
}

} /* mc_samples */

EXPORT_SINGLE_STATE("StabilizerStandingTrackCoM", mc_samples::StabilizerStandingTrackCoM)
