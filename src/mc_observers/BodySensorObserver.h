/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_observers/Observer.h>
#include <mc_observers/api.h>
#include <mc_rbdyn/Robot.h>

#include <SpaceVecAlg/SpaceVecAlg>

namespace mc_observers
{
/*!
 * BodySensorObserver is responsible for updating the state of the realRobot's
 * floating base from sensor measurements expressed as BodySensors. It
 * essentially replaces the old way of updating the realRobot (using `"UpdateRealFromSensors": true`).
 *
 * Sample configuration:
 * \code{.json}
 * "BodySensor":
 * {
 *   // Valid values are ["control", "estimator"]
 *   "UpdateFrom": "estimator",
 *   "UpdateFloatingBaseFromControl": false,
 *   "FloatingBaseSensorName": "FloatingBase",
 * }
 * \endcode
 */
struct MC_OBSERVER_DLLAPI BodySensorObserver : public Observer
{
  BodySensorObserver(const std::string & name, double dt, const mc_rtc::Configuration & config = {});
  void reset(const mc_control::MCController & ctl) override;
  /*! \brief  Compute the position of the floating base from either a
   * BodySensor. If the body-sensor is not directly attached to the floating
   * base link, the kinematic chain between the sensor and floating base is
   * taken into account. In this case, a prerequisite is that realRobot
   * kinematic parameters (encoder position and bodyPosW) are properly estimated (e.g by the EncoderEstimator)
   *
   * \param realRobot Real robot state. Requires realRobot kinematics to be
   * known.
   */
  bool run(const mc_control::MCController & ctl) override;
  /*! \brief Update realRobot from either BodySensor value or the control robot
   * depending on the update type.
   * Calls rbd::forwardKinematics and rbd::forwardVelocity
   *
   * \param realRobot robot to update
   */
  void updateRobot(const mc_control::MCController & ctl, mc_rbdyn::Robots & realRobots) override;

  void addToLogger(const mc_control::MCController & ctl, mc_rtc::Logger &) override;
  void removeFromLogger(mc_rtc::Logger &) override;
  void addToGUI(const mc_control::MCController & ctl, mc_rtc::gui::StateBuilder &) override;

protected:
  enum class Update
  {
    Control,
    Estimator
  };
  Update updateFrom_ = Update::Estimator;
  std::string fbSensorName_;
  sva::PTransformd posW_;
  sva::MotionVecd velW_;
};

} // namespace mc_observers
