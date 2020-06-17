#pragma once

#include <mc_control/fsm/State.h>

namespace mc_tasks
{
namespace lipm_stabilizer
{
struct StabilizerTask;
}
} // namespace mc_tasks

namespace mc_samples
{

struct StabilizerStandingTrackCoM : mc_control::fsm::State
{

    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;

private:
  std::shared_ptr<mc_tasks::lipm_stabilizer::StabilizerTask> stabilizerTask_ = nullptr;
  mc_rtc::Configuration config_;
};

} /* mc_samples */
