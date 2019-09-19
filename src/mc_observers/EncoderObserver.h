/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is heavily inspired by Stéphane Caron's lipm_walking_controller
 * https://github.com/stephane-caron/lipm_walking_controller
 */

#pragma once

#include <mc_observers/Observer.h>
#include <mc_observers/api.h>
#include <mc_rbdyn/Robot.h>

namespace mc_observers
{
/** Kinematics-only floating-base observer.
 *
 * See <https://scaron.info/teaching/floating-base-estimation.html> for
 * technical details on the derivation of this simple estimator.
 *
 */
struct MC_OBSERVER_DLLAPI EncoderObserver : public Observer
{
  /** Initialize floating base observer.
   *
   * \param controlRobot Robot reference.
   *
   */
  EncoderObserver(const std::string & name, double dt, const mc_rtc::Configuration & config = {});

  /** Reset floating base estimate.
   *
   * \param robot Robot from which the initial pose is to be estimated
   *
   */
  void reset(const mc_rbdyn::Robot & robot) override;

  /** Update floating-base transform of real robot.
   *
   * \param realRobot Measured robot state, to be updated.
   *
   */
  bool run(const mc_rbdyn::Robot & realRobot) override;

  /** Write observed floating-base transform to the robot's configuration.
   *
   * \param robot Robot state to write to.
   *
   */
  void updateRobot(mc_rbdyn::Robot & robot) override;

  void addToLogger(mc_rtc::Logger &) override;
  void removeFromLogger(mc_rtc::Logger &) override;
  void addToGUI(mc_rtc::gui::StateBuilder &) override;
  void removeFromGUI(mc_rtc::gui::StateBuilder &) override;

protected:
  bool posFromControl_;
  bool posFromSensor_;

  bool velFromControl_;
  bool velFromSensor_;
  bool velFromEstimation_;

  bool logEstimation_;

  std::vector<double> prevEncoders_;
  std::vector<double> encodersVelocity_;
};

} // namespace mc_observers
