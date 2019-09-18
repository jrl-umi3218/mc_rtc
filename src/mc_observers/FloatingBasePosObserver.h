/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is heavily inspired by Stéphane Caron's lipm_walking_controller
 * https://github.com/stephane-caron/lipm_walking_controller
 */

#pragma once

#include <mc_observers/Observer.h>
#include <mc_observers/api.h>
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
struct MC_OBSERVER_DLLAPI FloatingBasePosObserver : public Observer
{
  /** Initialize floating base observer.
   *
   * \param controlRobot Robot reference.
   *
   */
  FloatingBasePosObserver(const std::string & name, double dt, const mc_rtc::Configuration & config = {});
  ~FloatingBasePosObserver() override;

  /** Get anchor frame of a robot for a given contact state.
   *
   * \param robot Robot state to read frames from.
   *
   */
  sva::PTransformd getAnchorFrame(const mc_rbdyn::Robot & robot);

  /** Reset floating base estimate.
   *
   * \param robot Robot from which the initial pose is to be estimated
   *
   */
  void reset(const mc_rbdyn::Robot & robot) override;

  /** Update floating-base transform of real robot.
   *
   * \param realRobot Measured robot state, to be updated.
   *
   */
  bool run(const mc_rbdyn::Robot & realRobot) override;

  /** Write observed floating-base transform to the robot's configuration.
   *
   * \param robot Robot state to write to.
   *
   */
  void updateRobot(mc_rbdyn::Robot & realRobot) override;

  void updateBodySensor(mc_rbdyn::Robot & robot, const std::string & sensorName = "FloatingBase");

  /** Set fraction of total weight sustained by the left foot.
   *
   * \note This field is used in anchor frame computations.
   *
   */
  void leftFootRatio(double ratio)
  {
    leftFootRatio_ = ratio;
  }

  /** Get floating-base pose in the world frame.
   *
   */
  sva::PTransformd posW() const
  {
    return {orientation_, position_};
  }

  void addToLogger(mc_rtc::Logger &) override;
  void removeFromLogger(mc_rtc::Logger &) override;
  void addToGUI(mc_rtc::gui::StateBuilder &) override;
  void removeFromGUI(mc_rtc::gui::StateBuilder &) override;

private:
  /** Update floating-base orientation based on new observed gravity vector.
   *
   * \param realRobot Measured robot state.
   *
   */
  void estimateOrientation(const mc_rbdyn::Robot & realRobot);

  /* Update floating-base position.
   *
   * \param realRobot Measurements robot model.
   *
   * The new position is chosen so that the origin of the real anchor frame
   * coincides with the control anchor frame.
   *
   */
  void estimatePosition(const mc_rbdyn::Robot & realRobot);

private:
  Eigen::Matrix3d orientation_; /**< Rotation from world to floating-base frame */
  Eigen::Vector3d position_; /**< Translation of floating-base in world frame */
  double leftFootRatio_; /**< Fraction of total weight sustained by the left foot */
};

} // namespace mc_observers
