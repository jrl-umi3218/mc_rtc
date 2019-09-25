/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_observers/LowPassVelocityFilter.h>
#include <mc_observers/api.h>

#include "KinematicInertialPoseObserver.h"

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
  KinematicInertialObserver(const std::string & name, double dt, const mc_rtc::Configuration & config = {});

  /*! \brief  Resets the estimator from given robot state
   * Calls FLoatingBasePosObserver::reset(realRobot) to estimate the
   * floating base position, and initializes the velocity filter with robot's
   * floatingbase velocity realRobot.velW()
   *
   * \param realRobot Robot state from which the observer is to be initialized.
   */
  void reset(const mc_rbdyn::Robot & realRobot) override;

  /*! \brief  Resets the estimator from given robot state
   * First calls FLoatingBasePosObserver::reset(realRobot) to estimate the
   * floating base position, and initializes the velocity filter with the given
   * initial velocity
   *
   * \param realRobot Robot state from which the observer is to be initialized.
   * \param velW Initial velocity
   */
  void reset(const mc_rbdyn::Robot & realRobot, const sva::MotionVecd & velW);
  bool run(const mc_rbdyn::Robot & realRobot) override;
  void updateRobot(mc_rbdyn::Robot & realRobot) override;
  void updateBodySensor(mc_rbdyn::Robot & realRobot, const std::string & sensorName = "FloatingBase");

  /*! \brief Get floating-base velocity in the world frame.
   * The velocity is obtained by finite differences of the estimated position,
   * filtered with a simple low-pass filter.
   **/
  const sva::MotionVecd & velW() const;

  void addToLogger(mc_rtc::Logger &) override;
  void removeFromLogger(mc_rtc::Logger &) override;
  void addToGUI(mc_rtc::gui::StateBuilder &) override;
  void removeFromGUI(mc_rtc::gui::StateBuilder &) override;

private:
  /** Previous estimated position.
   * Used to compute finite differences estimation of the velocity */
  sva::PTransformd posWPrev_;

  /**
   * Estimated velocity through finite differences and low-pass filtering
   **/
  LowPassVelocityFilter<sva::MotionVecd> velFilter_;
  sva::MotionVecd velW_;

private:
  /** Prevent from resetting only the position */
  using KinematicInertialPoseObserver::reset;
};
} // namespace mc_observers
