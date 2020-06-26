#pragma once

#include <mc_control/fsm/State.h>
#include <mc_planning/generator.h>

namespace mc_samples
{

struct CoMTrajectoryGeneration_Initial : mc_control::fsm::State
{

  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  void updateSteps();

private:
  mc_planning::CenteredPreviewWindow preview_{0, 0};
  Eigen::Vector3d polesX_ = {1.0, 1.0, 150.0};
  Eigen::Vector3d polesY_ = {1.0, 1.0, 150.0};
  double previewTime_ = 1.6;

  unsigned iter_ = 0; ///< Number of iterations elapsed since started
  double t_ = 0; ///< Time elapsed since started
  double generationTime_ = 0; ///< Time spent generating the trajectory

  double delay_ = 0.5; ///< Delay before moving manually
  double transition_ = 0.2; ///< Transition time between steps (interpolation)

  Eigen::Vector3d leftFootPos_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d rightFootPos_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d feetCenterPos_ = Eigen::Vector3d::Zero();

private:
  std::shared_ptr<mc_planning::generator> comGenerator_;
  mc_planning::PreviewSteps<Eigen::Vector2d> steps_; ///< Foot steps defined as time, ZMP_x, ZMP_y
};

} // namespace mc_samples
