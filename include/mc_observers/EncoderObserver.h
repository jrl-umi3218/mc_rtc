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

  /** Configure observer */
  void configure(const mc_control::MCController & /*ctl*/, const mc_rtc::Configuration & /*config*/) override;

  /** Reset finite differences estimator from current encoder values and sets encoder velocity to zero
   *
   * \param ctl Controller calling this observer
   *
   * \throw std::runtime_error if the robot does not have encoder values
   */
  void reset(const mc_control::MCController & ctl) override;

  /*! \brief Computes encoder velocity by finite differences of position */
  bool run(const mc_control::MCController & ctl) override;

  /** Update the real robot from the estimator state, depending on its
   * configuration:
   * - UpdatePosition::Control : update joint position from robot().mbc().q
   * - UpdatePosition::Estimator : update joint position from
   *   robot().encoderValues()
   *
   * - UpdateVelocity::Control : update joint velocity from robot().mbc().alpha
   * - UpdateVelocity::Estimator : update joint velocity from finite differences
   *   of robot().encoderValues
   *
   * - UpdatePosition::None : joint position is not changed by this estimator
   * - UpdateVelocity::None : joint velocity is not changed by this estimator
   *
   * \param realRobots Real robots state to write to.
   *
   */
  void updateRobots(mc_control::MCController & ctl) override;

  void addToLogger(mc_control::MCController &, const std::string & /* category */ = "") override;
  void removeFromLogger(mc_control::MCController &, const std::string & /* category */ = "") override;

  const std::string type() const override
  {
    return "Encoder";
  }

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

  bool logEstimation_ = false;

  std::vector<double> prevEncoders_;
  std::vector<double> encodersVelocity_;
  std::string robot_;
};

} // namespace mc_observers
