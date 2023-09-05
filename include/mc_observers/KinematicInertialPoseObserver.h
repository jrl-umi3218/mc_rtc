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

  /** Reset floating base estimate from the current real robot state */
  void reset(const mc_control::MCController & ctl) override;

  /** Update floating-base estimation of real robot (IMU + kinematics)
   *
   * Requires an anchor frame to be provided as a datastore callback. This
   * should be a frame in-between the robot contacts, and its trajectory should
   * be continuous. See https://jrl-umi3218.github.io/mc_rtc/tutorials/recipes/observers.html
   * for futher information.
   *
   * Example:
   *
   * \code{cpp}
   * datastore().make_call("KinematicAnchorFrame::" + robot().name(), [this](const mc_rbdyn::Robot & robot) {
   *   return sva::interpolate(robot.surfacePose(leftFootSurface_), robot.surfacePose(rightFootSurface_), 0.5);
   * });
   * \endcode
   *
   * \see estimateOrientation(const mc_rbdyn::Robot & robot, const mc_rbdyn::Robot & realRobot);
   * \see estimatePosition(const mc_control::MCController & ctl);
   **/
  bool run(const mc_control::MCController & ctl) override;

  /** Write observed floating-base transform to the robot's configuration */
  void update(mc_control::MCController & ctl) override;

  /*! \brief Get floating-base pose in the world frame. */
  const sva::PTransformd & posW() const { return pose_; }

protected:
  void addToLogger(const mc_control::MCController & ctl, mc_rtc::Logger &, const std::string & category) override;
  void addToGUI(const mc_control::MCController &,
                mc_rtc::gui::StateBuilder &,
                const std::vector<std::string> & category) override;

  /** Compute floating-base orientation based on new observed gravity vector.
   *
   * \param robot Control robot model.
   * \param realRobot Measured robot state.
   *
   * \note Prior to mc_rtc 1.5, there was an important bug in the implementation
   * concerning how roll and pitch from sensor measurement was merged with the
   * yaw from control. This bug was fixed in mc_rtc 1.5, you might experience
   * behaviour changes if you were using the old implementation.
   */
  void estimateOrientation(const mc_rbdyn::Robot & robot, const mc_rbdyn::Robot & realRobot);

  /** Compute floating-base position from the estimated orientation and a
   * kinematic anchor frame
   *
   * \param robot Control robot model.
   * \param realRobot Measurements robot model.
   *
   * The new position is chosen so that the origin of the real anchor frame
   * coincides with the control anchor frame.
   */
  void estimatePosition(const mc_control::MCController & ctl);

protected:
  std::string robot_; /**< Robot to observe (default main robot) */
  std::string realRobot_; /**< Corresponding real robot (default main real robot) */
  std::string imuSensor_; /**< BodySensor containting IMU readings */

  std::string anchorFrameFunction_ = ""; ///< Name of datastore entry for the anchor frame function
  sva::PTransformd X_0_anchorFrame_ =
      sva::PTransformd::Identity(); ///< Control anchor frame (provided through the datastore)
  sva::PTransformd X_0_anchorFrameReal_ =
      sva::PTransformd::Identity(); ///< Real anchor frame (provided through the datastore)

  double maxAnchorFrameDiscontinuity_ =
      0.01; ///< Threshold (norm) above wich the anchor frame is considered to have had a discontinuity
  bool anchorFrameJumped_ = false; /** Detects whether the anchor frame had a discontinuity */
  bool firstIter_ = true; /** Ignore anchor frame check on first iteration */

private:
  sva::PTransformd pose_ = sva::PTransformd::Identity(); ///< Estimated pose of the floating-base in world frame */
  bool showAnchorFrame_ = false; /**< Whether to show the anchor frame in the GUI */
  bool showAnchorFrameReal_ = false; /**< Whether to show the anchor frame in the GUI */
  bool showPose_ = false; /**< Whether to show the anchor frame in the GUI */
  bool advancedGUI_ = false; ///< When true, displays an Advanced tab in the GUI

  bool logPose_ = true; ///< Whether to log the estimated pose
  bool logAnchorFrame_ = true; ///< Whether to log the user-provided anchor frame (control and real)
};

} // namespace mc_observers
