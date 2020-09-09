/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_filter/LowPass.h>
#include <mc_observers/KinematicInertialPoseObserver.h>
#include <mc_observers/api.h>

namespace mc_observers
{
/** Kinematics-inertial observer of the floating base position with finite
 * differences estimation of the floating base velocity using low-pass
 * filtering.
 *
 * \see KinematicInertialPoseObserver for details about the estimation of the floating
 * base position.
 */
struct MC_OBSERVER_DLLAPI KinematicInertialObserver : public KinematicInertialPoseObserver
{
  KinematicInertialObserver(const std::string & type, double dt)
  : KinematicInertialPoseObserver(type, dt), velFilter_(dt, 2 * dt)
  {
  }

  void configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config) override;

  /*! \brief  Resets the estimator from given robot state
   * Calls FLoatingBasePosObserver::reset(realRobot) to estimate the
   * floating base position, and initializes the velocity filter with robot's
   * floatingbase velocity realRobot.velW()
   *
   * \param ctl Controller access
   */
  void reset(const mc_control::MCController & ctl) override;

  /*! \brief  Resets the estimator from given robot state
   * First calls FLoatingBasePosObserver::reset(realRobot) to estimate the
   * floating base position, and initializes the velocity filter with the given
   * initial velocity
   *
   * \param realRobot Robot state from which the observer is to be initialized.
   * \param velW Initial velocity
   */
  void reset(const mc_control::MCController & ctl, const sva::MotionVecd & velW);
  bool run(const mc_control::MCController & ctl) override;
  void update(mc_control::MCController & ctl) override;

  /*! \brief Get floating-base velocity in the world frame.
   * The velocity is obtained by finite differences of the estimated position,
   * filtered with a simple low-pass filter.
   **/
  const sva::MotionVecd & velW() const;

protected:
  void addToLogger(const mc_control::MCController & ctl, mc_rtc::Logger &, const std::string & category) override;
  void removeFromLogger(mc_rtc::Logger &, const std::string & category) override;
  void addToGUI(const mc_control::MCController &,
                mc_rtc::gui::StateBuilder &,
                const std::vector<std::string> & category) override;

private:
  bool showVelocity_ = true;
  mc_rtc::gui::ArrowConfig velocityArrowConfig_;

  /** Previous estimated position.
   * Used to compute finite differences estimation of the velocity */
  sva::PTransformd posWPrev_ = sva::PTransformd::Identity();

  /**
   * Estimated velocity through finite differences and low-pass filtering
   **/
  mc_filter::LowPass<sva::MotionVecd> velFilter_;
  sva::MotionVecd velW_ = sva::MotionVecd::Zero();

  bool logVelocity_ = true; ///< Whether to log the estimated velocity

private:
  /** Prevent from resetting only the position */
  using KinematicInertialPoseObserver::reset;
};
} // namespace mc_observers
