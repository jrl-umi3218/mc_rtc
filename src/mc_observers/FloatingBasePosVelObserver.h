/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is heavily inspired by Stéphane Caron's lipm_walking_controller
 * https://github.com/stephane-caron/lipm_walking_controller
 */

#pragma once

#include <mc_observers/api.h>
#include <mc_observers/LowPassVelocityFilter.h>
#include "FloatingBasePosObserver.h"

namespace mc_observers
{
struct MC_OBSERVER_DLLAPI FloatingBasePosVelObserver : public FloatingBasePosObserver
{
  FloatingBasePosVelObserver(const std::string& name, double dt, const mc_rtc::Configuration & config = {});
  ~FloatingBasePosVelObserver() override;

  /*!
   * @brief  Resets the estimator from given robot state
   * Calls FLoatingBasePosObserver::reset(realRobot) to estimate the
   * floating base position, and initializes the velocity filter with robot's
   * floatingbase velocity realRobot.velW()
   *
   * @param realRobot Robot state from which the observer is to be initialized.
   * @param velW Initial velocity
   */
  void reset(const mc_rbdyn::Robot & controlRobot, const mc_rbdyn::Robot & realRobot) override;

  /*!
   * @brief  Resets the estimator from given robot state
   * First calls FLoatingBasePosObserver::reset(realRobot) to estimate the
   * floating base position, and initializes the velocity filter with the given
   * initial velocity
   *
   * @param realRobot Robot state from which the observer is to be initialized.
   * @param velW Initial velocity
   */
  void reset(const mc_rbdyn::Robot & controlRobot, const mc_rbdyn::Robot & realRobot, const sva::MotionVecd & velW);
  bool run(const mc_rbdyn::Robot & controlRobot, const mc_rbdyn::Robot & realRobot) override;
  void updateRobot(mc_rbdyn::Robot & robot) override;
  void updateBodySensor(mc_rbdyn::Robot & robot, const std::string & sensorName = "FloatingBase");

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
  using FloatingBasePosObserver::reset;
};
} // namespace mc_observers
