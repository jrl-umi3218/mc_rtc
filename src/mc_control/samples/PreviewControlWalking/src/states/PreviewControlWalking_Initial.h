#pragma once

#include "../PreviewControlWalking.h"

#include <mc_control/fsm/State.h>
#include <mc_planning/FootstepManager.h>
#include <mc_planning/PreviewControl.h>
#include <mc_tasks/SurfaceTransformTask.h>
#include <mc_tasks/lipm_stabilizer/StabilizerTask.h>

namespace internal
{
/*! \brief See
 * https://github.com/jrl-umi3218/mc_rtc/blob/29b471e7ef317eca3358327c664196ee657a8ab4/include/mc_tasks/lipm_stabilizer/StabilizerTask.h#L664-L671
 */
struct EnumClassHash
{
  template<typename T>
  std::size_t operator()(T t) const
  {
    return static_cast<std::size_t>(t);
  }
};
} // namespace internal

struct PreviewControlWalking_Initial : mc_control::fsm::State
{
  PreviewControlWalking_Initial();

  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  void setContacts(PreviewControlWalking & ctl,
                   const std::vector<std::pair<mc_tasks::lipm_stabilizer::ContactState, sva::PTransformd>> & contacts,
                   bool fullDoF = false);

  int previewSize()
  {
    // Preview size should be same for x and y
    return previewControl_[0].previewSize();
  }

  mc_rbdyn::lipm_stabilizer::StabilizerConfiguration stabilizerConfig_;
  std::shared_ptr<mc_tasks::lipm_stabilizer::StabilizerTask> stabilizerTask_;

  std::unordered_map<mc_planning::Foot, std::shared_ptr<mc_tasks::SurfaceTransformTask>, internal::EnumClassHash>
      swingFootTasks_;

  double horizon_ = 2.0;

  mc_planning::FootstepManager footstepManager_;
  std::vector<mc_planning::PreviewControl> previewControl_;

  Eigen::Matrix3Xd refZmpTraj_;

  mc_planning::SupportPhase supportPhase_;
};
