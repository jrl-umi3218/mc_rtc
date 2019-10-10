/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is  inspired by Stephane's Caron implementation as part of
 * lipm_walking_controller <https://github.com/stephane-caron/lipm_walking_controller>
 */

#pragma once

#include <mc_observers/Observer.h>
#include <mc_observers/api.h>
#include <mc_rbdyn/Robot.h>

#include <SpaceVecAlg/SpaceVecAlg>

namespace mc_observers
{
/** Kinematics-inertial observer of the floating base position.
 *
 * This estimator relies on the feet contact location and the IMU orientation to
 * estimate the floating base position and orientation. Note that this is a very
 * simple kinematic inertial estimator that does not take advantage of
 * closed-loop kinematic chains. As a result, the estimate using the left foot
 * contact as an anchor point and the one using the right foot might differ
 * slightly.
 *
 * See <https://scaron.info/teaching/floating-base-estimation.html> for
 * technical details on the derivation of this simple estimator.
 *
 * and for comparison between simple kinematic-inertial estimators the paper
 *
 * "Experimental Evaluation of Simple Estimators for Humanoid Robots"
 * by Thomas Flayols, Andrea Del Prete, Patrick Wensing, Alexis Mifsud, Mehdi
Benallegue, Olivier Stasse
 * <https://hal.archives-ouvertes.fr/hal-01574819/document>
 */
struct MC_OBSERVER_DLLAPI KinematicInertialPoseObserver : public Observer
{
  /*! Initialize floating base observer */
  KinematicInertialPoseObserver(const std::string & name, double dt, const mc_rtc::Configuration & config = {});

  /** Reset floating base estimate from the current control robot state and
   * estimates the floating base position by calling run()
   */
  void reset(const mc_control::MCController & ctl) override;

  /** Update floating-base transform of real robot
   */
  bool run(const mc_control::MCController & ctl) override;

  /** Write observed floating-base transform to the robot's configuration
   *
   * \param robot Robot state to write to
   *
   */
  void updateRobots(const mc_control::MCController & ctl, mc_rbdyn::Robots & realRobots) override;

  void updateBodySensor(mc_rbdyn::Robots & robots, const std::string & sensorName = "FloatingBase");

  /*! \brief Get floating-base pose in the world frame. */
  sva::PTransformd posW() const
  {
    return {orientation_, position_};
  }

  void addToLogger(const mc_control::MCController & ctl, mc_rtc::Logger &) override;
  void removeFromLogger(mc_rtc::Logger &) override;

protected:
  /** Update floating-base orientation based on new observed gravity vector.
   *
   * \param robot Control robot model.
   * \param realRobot Measured robot state.
   *
   */
  void estimateOrientation(const mc_rbdyn::Robot & robot, const mc_rbdyn::Robot & realRobot);

  /* Update floating-base position.
   *
   * \param robot Control robot model.
   * \param realRobot Measurements robot model.
   *
   * The new position is chosen so that the origin of the real anchor frame
   * coincides with the control anchor frame.
   *
   */
  void estimatePosition(const mc_control::MCController & ctl);

private:
  Eigen::Matrix3d orientation_; /**< Rotation from world to floating-base frame */
  Eigen::Vector3d position_; /**< Translation of floating-base in world frame */
  double leftFootRatio_; /**< Fraction of total weight sustained by the left foot */
};

} // namespace mc_observers
