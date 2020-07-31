/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is inspired by Stephane's Caron implementation as part of
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
  KinematicInertialPoseObserver(const std::string & type, double dt) : Observer(type, dt) {}

  /** Configure observer */
  void configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config) override;

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
  void updateRobots(mc_control::MCController & ctl) override;

  /*! \brief Get floating-base pose in the world frame. */
  sva::PTransformd posW() const
  {
    return {orientation_, position_};
  }

protected:
  void addToLogger(mc_control::MCController & ctl, const std::string & category) override;
  void removeFromLogger(mc_control::MCController & ctl, const std::string & category) override;
  void addToGUI(mc_control::MCController &, const std::vector<std::string> & category) override;

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

protected:
  std::string robot_; /**< Robot to observe */
  std::string imuSensor_; /**< BodySensor containting IMU readings */

private:
  Eigen::Matrix3d orientation_ = Eigen::Matrix3d::Identity(); /**< Rotation from world to floating-base frame */
  Eigen::Vector3d position_ = Eigen::Vector3d::Zero(); /**< Translation of floating-base in world frame */
  bool showAnchorFrame_ = false; /**< Whether to show the anchor frames in the GUI */
  bool log_ = false; /**< Whether to log the estimation results */
};

} // namespace mc_observers
