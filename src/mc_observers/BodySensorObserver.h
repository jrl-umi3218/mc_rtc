/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_observers/api.h>
#include <mc_observers/Observer.h>
#include <mc_rbdyn/Robot.h>

#include <SpaceVecAlg/SpaceVecAlg>

namespace mc_observers
{
/** Kinematics-only floating-base observer.
 *
 * See <https://scaron.info/teaching/floating-base-estimation.html> for
 * technical details on the derivation of this simple estimator.
 *
 */
struct MC_OBSERVER_DLLAPI BodySensorObserver : public Observer
{
  BodySensorObserver(const std::string& name, double dt, const mc_rtc::Configuration & config = {});
  void reset(const mc_rbdyn::Robot & robot) override;
  bool run(const mc_rbdyn::Robot & robot) override;
  void updateRobot(mc_rbdyn::Robot & robot) override;

  void addToLogger(mc_rtc::Logger &) override;
  void removeFromLogger(mc_rtc::Logger &) override;
  void addToGUI(mc_rtc::gui::StateBuilder &) override;

 protected:
  bool updateFbFromSensor_;
  std::string fbSensorName_;
  bool updateFbFromControl_;
  bool updateEncoderPosFromControl_;
  bool updateEncoderVelFromControl_;
  bool updateEncoderPosFromSensor_;
  bool updateEncoderVelFromSensor_;
  sva::PTransformd posW_;
  sva::MotionVecd velW_;
};

} // namespace mc_observers
