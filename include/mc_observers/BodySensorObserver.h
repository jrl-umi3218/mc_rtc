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
/*! The BodySensorObserver is responsible for estimating the state of a robot's
 * floating base from sensor measurements provided by a BodySensor and the
 * kinematics beween this sensor and the floating base.
 * It is assumed here that the floating base sensor kinematics estimate are synchronized.
 *
 * \see BodySensorObserver::run() for usage requirements
 *
 * The default configuration estimates the floating base pose from the main bodysensor (typically an IMU).
 */
struct MC_OBSERVER_DLLAPI BodySensorObserver : public Observer
{
  BodySensorObserver(const std::string & type, double dt) : Observer(type, dt) {}

  /** Configure observer */
  void configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & /*config*/) override;

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
   * - Update::Sensor: Computes the position of the floating base from a BodySensor and the kinematic chain
   *   between it and the floating base. If the BodySensor is not directly attached to the floating base link, this
   * estimator requires accurate estimation of the robot kinematics. A typical pipeline will achieve this by running the
   * EncoderObserver observer before the BodySensorObserver.
   *
   *   It is assumed here that the floating base sensor and encoders are synchronized.
   *
   * \param ctl The controller instance running this observer
   */
  bool run(const mc_control::MCController & ctl) override;

  /*! \brief Update the robot's floating base from its estimated pose
   *
   * \see run for usage requirements
   *
   * \note Calls rbd::forwardKinematics and rbd::forwardVelocity
   */
  void update(mc_control::MCController & ctl) override;

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

protected:
  void addToLogger(const mc_control::MCController &, mc_rtc::Logger &, const std::string & category) override;
  void removeFromLogger(mc_rtc::Logger &, const std::string & category) override;
  void addToGUI(const mc_control::MCController &,
                mc_rtc::gui::StateBuilder &,
                const std::vector<std::string> & category) override;

protected:
  enum class Update
  {
    Control, ///< Use the control robot floating base state
    Sensor ///< Use the body sensor to determine the floating base pose
  };
  Update updateFrom_ = Update::Sensor;
  std::string fbSensorName_;
  sva::PTransformd posW_ = sva::PTransformd::Identity();
  sva::MotionVecd velW_ = sva::MotionVecd::Zero();
  sva::MotionVecd accW_ = sva::MotionVecd::Zero();
  std::string robot_;
  std::string updateRobot_;
  bool updatePose_ = true;
  bool updateVel_ = true;

  bool logPos_ = true;
  bool logVel_ = true;
  bool logAcc_ = true;
  bool guiPos_ = false;
  bool guiVel_ = true;
  mc_rtc::gui::ArrowConfig guiVelConfig_;
  bool guiAcc_ = false;
  mc_rtc::gui::ArrowConfig guiAccConfig_;
  bool advancedGUI_ = false; ///< When true, displays an Advanced tab in the GUI
};

} // namespace mc_observers
