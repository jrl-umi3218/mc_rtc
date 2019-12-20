/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/fsm/State.h>
#include <mc_planning/Pendulum.h>
#include <mc_tasks/lipm_stabilizer/Contact.h>
#include <mc_tasks/lipm_stabilizer/StabilizerTask.h>

namespace mc_control
{
namespace fsm
{

/**
 * @brief Simple state that manages the LIPMStabilizer for a standing biped
 *
 * - Allows to control the CoM position in-between Left and Right foot contacts
 * - Provides a simple example for generating dynamic trajectories of the CoM
 */
struct StabilizerStandingState : State
{
  StabilizerStandingState();

  void configure(const mc_rtc::Configuration & config) override;

  void start(Controller &) override;

  bool run(Controller &) override;

  void teardown(Controller &) override;

protected:
  void target(const Controller &, double leftFootRatio);

protected:
  std::shared_ptr<mc_tasks::lipm_stabilizer::StabilizerTask> stabilizerTask_ = nullptr;
  mc_rtc::Configuration config_;

  double hasCompletion_ = false;
  Eigen::Vector3d dcmThreshold_ = Eigen::Vector3d{0.01, 0.01, 0.01};

  mc_tasks::lipm_stabilizer::ContactState contactState_ = mc_tasks::lipm_stabilizer::ContactState::DoubleSupport;

  mc_planning::Pendulum pendulum_; /** LIPM Pendulum model */

  /** Interpolation ratio between left and right foot:
   * - 0: left foot
   * - 1: right foot
   */
  double leftFootRatio_ = 0.5;

  double K_ = 5; /** CoM tracking stiffness (set-point) */
  double D_ = 0.;
  Eigen::Vector3d copTarget_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d comTarget_ = Eigen::Vector3d::Zero();
};

} // namespace fsm
} // namespace mc_control
