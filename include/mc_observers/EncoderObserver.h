/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_observers/Observer.h>
#include <mc_observers/api.h>
#include <mc_rbdyn/Robot.h>

namespace mc_observers
{
/*! Estimator of floating base position and velocity
 *
 * Position is directly obtained from sensor values
 * Velocity is then computed by finite differences
 *
 * The default behaviour is to update the real robot from the estimated position and velocity.
 *
 * You can configure the estimator with the following JSON configuration
 *
 * \code{.json}
 * "Encoder":
 * {
 *   // Valid values are ["estimator", "control", "none"]
 *   "UpdatePosition": "estimator",
 *   // Valid values are ["estimator", "control", "none"]
 *   "UpdateVelocity": "estimator",
 *   "Log" : true
 * }
 * \endcode
 */
struct MC_OBSERVER_DLLAPI EncoderObserver : public Observer
{
  EncoderObserver(const std::string & type, double dt) : Observer(type, dt) {}

  /** Configure observer */
  void configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config) override;

  /** Reset finite differences estimator from current encoder values and sets encoder velocity to zero
   *
   * \throw std::runtime_error if the robot does not have encoder values
   */
  void reset(const mc_control::MCController & ctl) override;

  /*! \brief Computes encoder velocity if necessary:
   *
   * - If VelUpdate::EncoderFiniteDifferences, computes velocity from finite
   *   differences of encoder position
   **/
  bool run(const mc_control::MCController & ctl) override;

  /** Update the real robot from the estimator state according to the observer's
   * results
   *
   * \see PosUpdate, VelUpdate for details on the method used for estimation
   */
  void update(mc_control::MCController & ctl) override;

protected:
  void addToLogger(const mc_control::MCController &, mc_rtc::Logger &, const std::string &) override;

protected:
  /*! Position update type */
  enum class PosUpdate
  {
    Control, ///< Use joint value from robot.mbc.q (control)
    EncoderValues, ///< Encoder values from robot.encoderValues (encoder sensor)
    None ///< Do not compute/update value
  };
  /*! Velocity update type */
  enum class VelUpdate
  {
    Control, ///< Use joint velocities from robot.mbc.alpha (control)
    EncoderVelocities, ///< Joint velocity from robot.encoderVelocities (encoder velocity sensor)
    EncoderFiniteDifferences, ///< Joint velocity from finite differences of robot.encoderValues (encoder sensor)
    None ///< Do not compute/update value
  };

  PosUpdate posUpdate_ = PosUpdate::EncoderValues;
  VelUpdate velUpdate_ = VelUpdate::EncoderFiniteDifferences;
  bool computeFK_ = true; ///< Whether to compute forward kinematics
  bool computeFV_ = true; ///< Whether to compute forward velocity

  std::string robot_ = ""; ///< Robot estimated by this observer
  std::string updateRobot_ = ""; ///< Robot to update (defaults to robot_)

  std::vector<double> prevEncoders_; ///< Previous encoder values (for VelUpdate::EncoderFiniteDifferences)
  std::vector<double> encodersVelocity_; ///< Estimated encoder velocity

  bool logPosition_ = false;
  bool logVelocity_ = true;
};

} // namespace mc_observers
