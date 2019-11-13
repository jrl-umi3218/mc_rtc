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
/*! BodySensorObserver is responsible for updating the state of the realRobot's
 * floating base from sensor measurements expressed as BodySensors. It
 * essentially replaces the old way of updating the realRobot (using `"UpdateRealFromSensors": true`).
 *
 * \see BodySensorObserver::run() for usage requirements
 *
 * Default configuration estimates the floating base pose from the
 * "FloatingBase" bodysensor.
 *
 * \code{.json}
 * "BodySensor":
 * {
 *   // Valid values are ["control", "estimator"]
 *   "UpdateFrom": "estimator",
 *   "FloatingBaseSensorName": "FloatingBase",
 * }
 * \endcode
 */
struct MC_OBSERVER_DLLAPI BodySensorObserver : public Observer
{
  BodySensorObserver(const std::string & name, double dt, const mc_rtc::Configuration & config = {});

  /** \brief Resets the observer.
   *
   * Default implementation ensures that the BodySensor provided in the
   * configuration exists and compute the initial floating base pose (calls run())
   *
   * \param ctl The controller running this observer
   */
  void reset(const mc_control::MCController & ctl) override;

  /*! \brief  Determines the pose of the floating base
   *
   * The pose of the floating base is determined depending on the update type
   * chosen:
   * - Update::Control: copies the floating base position from the control robot
   *   (no estimation)
   * - Update::Estimator: Computes the position of the floating base from a BodySensor and the kinematic chain
   *   between it and the floating base. If the BodySensor is not directly
   *   attached to the floating base link, this estimator requires accurate
   *   estimation of the real robot's forward kinematics. It is assumed here that the floating base sensor and encoders
   * are synchronized. A typical pipeline will achieve this by running the EncoderObserver observer before the
   * BodySensorObserver
   *
   * \param ctl The controller instance running this observer
   */
  bool run(const mc_control::MCController & ctl) override;

  /*! \brief Update realRobots floating base from its estimated pose
   *
   * \see run for usage requirements
   *
   * \note Calls rbd::forwardKinematics and rbd::forwardVelocity
   *
   * \param realRobots Current implementation updates realRobots.robot()
   */
  void updateRobots(const mc_control::MCController & ctl, mc_rbdyn::Robots & realRobots) override;

  /*! \brief Get floating-base pose in the world frame. */
  const sva::PTransformd & posW() const
  {
    return posW_;
  }

  /*! \brief Get floating-base velocity in the world frame. */
  const sva::MotionVecd & velW() const
  {
    return velW_;
  }

  void addToLogger(const mc_control::MCController & ctl, mc_rtc::Logger &) override;
  void removeFromLogger(mc_rtc::Logger &) override;
  void addToGUI(const mc_control::MCController & ctl, mc_rtc::gui::StateBuilder &) override;

protected:
  enum class Update
  {
    Control, ///< Use the control robot floating base state
    Estimator ///< Use the body sensor to determine the floating base pose
  };
  Update updateFrom_ = Update::Estimator;
  std::string fbSensorName_;
  sva::PTransformd posW_;
  sva::MotionVecd velW_;
};

} // namespace mc_observers
