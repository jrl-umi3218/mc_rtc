/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is inspired by Stephane's Caron implementation as part of
 * lipm_walking_controller <https://github.com/stephane-caron/lipm_walking_controller>
 */

#pragma once

#include <mc_observers/Observer.h>
#include <mc_observers/api.h>
#include <mc_rbdyn/Robot.h>
#include <mc_signal/LowPassFiniteDifferencesVelocityFilter.h>

namespace mc_observers
{

/**
 * @brief Estimates the CoM velocity by finite differences of its position,
 * as given by realRobot().com().
 *
 * Prerequisites:
 * - realRobot updated with accurate floating base position estimation and body
 *   kinematics (position)
 */
struct MC_OBSERVER_DLLAPI CoMVelocityObserver : public Observer
{
  /*! Initialize floating base observer */
  CoMVelocityObserver(const std::string & name, double dt, const mc_rtc::Configuration & config = {});

  /** Reset floating base estimate from the current control robot state and
   * estimates the floating base position by calling run()
   */
  void reset(const mc_control::MCController & ctl) override;

  /** Update floating-base transform of real robot
   */
  bool run(const mc_control::MCController & ctl) override;

  /** Write observed floating-base transform to the robot's configuration
   *
   * \param robot Robot state to write to
   *
   */
  void updateRobots(const mc_control::MCController & ctl, mc_rbdyn::Robots & realRobots) override;

  void addToLogger(const mc_control::MCController & ctl, mc_rtc::Logger &) override;
  void removeFromLogger(mc_rtc::Logger &) override;

private:
  mc_signal::LowPassFiniteDifferencesVelocityFilter<Eigen::Vector3d>
      comVelFilter_; /**< Low-pass filter used to estimate the CoM velocity */
  Eigen::Vector3d realComd_ = Eigen::Vector3d::Zero();
};

} // namespace mc_observers
