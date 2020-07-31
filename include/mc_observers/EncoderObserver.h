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

  /*! \brief Computes encoder velocity by finite differences of position
   *
   * \see PosUpdate
   * \see VelUpdate
   **/
  bool run(const mc_control::MCController & ctl) override;

  /** Update the real robot from the estimator state according to the observer's
   * results
   *
   * \see PosUpdate, VelUpdate for details on the data used for estimatation
   */
  void updateRobots(mc_control::MCController & ctl) override;

protected:
  void addToLogger(mc_control::MCController & ctl, const std::string & category) override;
  void removeFromLogger(mc_control::MCController & ctl, const std::string & category) override;

protected:
  /*! \brief Update source for the update. */
  enum class PosUpdate
  {
    Control, ///< Use joint value from robot.mbc.alpha (control)
    EncoderValues, ///< Encoder values from robot.encoderValues (encoder sensor)
    None ///< Do not compute/update value
  };
  enum class VelUpdate
  {
    Control, ///< Use joint velocities from robot.mbc.alpha (control)
    EncoderVelocities, ///< Joint velocity from robot.encoderValues (encoder sensor)
    EncoderFiniteDifferences, ///< Joint velocity from finite differences of robot.encoderValues (encoder sensor)
    None ///< Do not compute/update value
  };

  PosUpdate posUpdate_ = PosUpdate::EncoderValues;
  VelUpdate velUpdate_ = VelUpdate::EncoderFiniteDifferences;

  std::string robot_;

  std::vector<double> prevEncoders_;
  std::vector<double> encodersVelocity_;
};

} // namespace mc_observers
