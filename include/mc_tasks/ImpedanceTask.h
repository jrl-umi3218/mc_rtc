/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/ImpedanceGains.h>
#include <mc_tasks/SurfaceTransformTask.h>

#include <mc_filter/LowPass.h>

#include <mc_rtc/constants.h>

namespace mc_tasks
{

namespace force
{

/*! \brief Impedance control of the end-effector.
 *
 *  ImpedanceTask passes the following "compliance" position and orientation (i.e., \f$ p_c \f$)
 *  to the target of the SurfaceTransformTask, which is the base class of this class.
 *
 *  \f[
 *      M \Delta \ddot{p}_{cd} + D \Delta \dot{p}_{cd} + K \Delta p_{cd} = K_f (f_m - f_d)
 *      {\rm where} \Delta p_{cd} = p_c - p_d
 *  \f]
 *  where \f$ p_* \f$ is the end-effector position and orientation, and \f$ f_* \f$ is the end-effector wrench.
 *  Subscripts \f$ d, c, m \f$ mean the desired, compliance, and measured values, respectively.
 *  \f$ M, D, K \f$ are the mass, damper, and spring parameters of the impedance, respectively.
 *  \f$ K_f \f$ is the wrench gain.
 *
 *  Desired values \f$ p_d, \dot{p}_d, \ddot{p}_d, f_d \f$ are given from the user.
 *  The measured value \f$ f_m \f$ is obtained from the sensor.
 *
 *  In the SurfaceTransformTask, the "IK" acceleration (i.e., \f$ \ddot{p}_{IK} \f$) are calculated
 *  and passed to the acceleration-level IK.
 *
 * \f[
 *     \ddot{p}_{IK} = \ddot{p}_{c} + K_d ( \dot{p}_c - \dot{p}_a ) + K_s ( p_c - p_a )
 * \f]
 *
 *  \f$ K_s, K_d \f$ are the stiffness and damping parameters, respectively.
 *  Subscripts \f$ a \f$ means actual value.
 *
 *  Reference:
 *    Bruno Siciliano and Luigi Villani, Robot Force Control, Springer, 1999
 *    https://www.springer.com/jp/book/9780792377337
 *
 */
struct MC_TASKS_DLLAPI ImpedanceTask : SurfaceTransformTask
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*! \brief Constructor.
   *
   * \param surfaceName Name of the surface frame to control
   *
   * \param robots Robots controlled by this task
   *
   * \param robotIndex Index of the robot controlled by this task
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   *
   * \throws If the body the task is attempting to control does not have a
   * sensor attached to it
   *
   */
  ImpedanceTask(const std::string & surfaceName,
                const mc_rbdyn::Robots & robots,
                unsigned robotIndex,
                double stiffness = 5.0,
                double weight = 1000.0);

  /*! \brief Reset the task
   *
   * Set the target and compliance poses to the current surface, and reset the target and compliance
   * velocity and acceleration to zero.
   *
   */
  void reset() override;

  /*! \brief Access the impedance gains */
  inline const ImpedanceGains & gains() const noexcept
  {
    return gains_;
  }

  /*! \brief Access the impedance gains */
  inline ImpedanceGains & gains() noexcept
  {
    return gains_;
  }

  /*! \brief Get the target pose of the surface in the world frame. */
  const sva::PTransformd & targetPose() const noexcept
  {
    return targetPoseW_;
  }

  /*! \brief Set the target pose of the surface in the world frame. */
  void targetPose(const sva::PTransformd & pose)
  {
    targetPoseW_ = pose;
  }

  /*! \brief Get the target velocity of the surface in the world frame. */
  const sva::MotionVecd & targetVel() const noexcept
  {
    return targetVelW_;
  }

  /*! \brief Set the target velocity of the surface in the world frame. */
  void targetVel(const sva::MotionVecd & vel)
  {
    targetVelW_ = vel;
  }

  /*! \brief Get the target acceleration of the surface in the world frame. */
  const sva::MotionVecd & targetAccel() const noexcept
  {
    return targetAccelW_;
  }

  /*! \brief Set the target acceleration of the surface in the world frame. */
  void targetAccel(const sva::MotionVecd & accel)
  {
    targetAccelW_ = accel;
  }

  /*! \brief Get the relative pose from target frame to compliance frame represented in the world frame. */
  const sva::PTransformd & deltaCompliancePose() const
  {
    return deltaCompPoseW_;
  }

  /*! \brief Get the compliance pose of the surface in the world frame.
   *
   *  \note Compliance pose cannot be set by user because it is calculated from the impedance equation internally.
   *  See the Constructor description for the definition of compliance pose.
   *
   */
  const sva::PTransformd compliancePose() const
  {
    sva::PTransformd T_0_d(targetPoseW_.rotation());
    return T_0_d * deltaCompPoseW_ * T_0_d.inv() * targetPoseW_;
  }

  /*! \brief Get the target wrench in the surface frame. */
  const sva::ForceVecd & targetWrench() const noexcept
  {
    return targetWrench_;
  }

  /*! \brief Set the target wrench in the world frame.
   * This function will convert the wrench from the world frame to the surface frame, and call targetWrench().
   *
   */
  void targetWrenchW(const sva::ForceVecd & wrenchW)
  {
    const auto & X_0_s = robots.robot(rIndex).surfacePose(surfaceName);
    targetWrench(X_0_s.dualMul(wrenchW));
  }

  /*! \brief Set the target wrench in the surface frame. */
  void targetWrench(const sva::ForceVecd & wrench)
  {
    targetWrench_ = wrench;
  }

  /*! \brief Get the measured wrench in the surface frame. */
  const sva::ForceVecd & measuredWrench() const
  {
    return measuredWrench_;
  }

  /*! \brief Get the filtered measured wrench in the surface frame. */
  const sva::ForceVecd & filteredMeasuredWrench() const
  {
    return filteredMeasuredWrench_;
  }

  /*! \brief Get the cutoff period for the low-pass filter of measured wrench. */
  double cutoffPeriod() const
  {
    return lowPass_.cutoffPeriod();
  }

  /*! \brief Set the cutoff period for the low-pass filter of measured wrench. */
  void cutoffPeriod(double cutoffPeriod)
  {
    lowPass_.cutoffPeriod(cutoffPeriod);
  }

  /*! \brief Get whether hold mode is enabled. */
  inline bool hold() const noexcept
  {
    return hold_;
  }

  /*! \brief Set hold mode.
   *
   *  In hold mode, the compliance modification (deltaCompPoseW_) is automatically updated so that the final target pose
   * sent to the QP (i.e. compliancePose()) remains constant even if the user-specified target pose changes. A typical
   * use case for hold mode is to set the current end-effector pose as targetPose. Thanks to the hold mode, the robot
   * does not move the end-effector pose, but the deltaCompPoseW_ becomes smaller, and the external force exerted by the
   * spring term of impedance dynamics becomes smaller. Without the hold mode, the mass and damper effects of impedance
   * dynamics would cause the compliancePose to temporarily deviate from the commanded targetPose, causes unintended
   * movement of the end-effector.
   */
  inline void hold(bool hold) noexcept
  {
    hold_ = hold;
  }

  /*! \brief Load parameters from a Configuration object. */
  void load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) override;

protected:
  ImpedanceGains gains_ = ImpedanceGains::Default();

  /** Relative pose, velocity, and acceleration from target frame to compliance frame represented in the world frame.
   *  To store these values across control cycles, represent them in a constant world frame instead of the time-varying
   *  surface frame.
   *  @{
   */
  sva::PTransformd deltaCompPoseW_ = sva::PTransformd::Identity();
  sva::MotionVecd deltaCompVelW_ = sva::MotionVecd::Zero();
  sva::MotionVecd deltaCompAccelW_ = sva::MotionVecd::Zero();
  /// @}

  // Limits of relative pose, velocity, and acceleration from target frame to compliance frame.
  double deltaCompPoseLinLimit_ = 1.0;
  double deltaCompPoseAngLimit_ = mc_rtc::constants::PI;
  double deltaCompVelLinLimit_ = 1e3;
  double deltaCompVelAngLimit_ = 1e3;
  double deltaCompAccelLinLimit_ = 1e3;
  double deltaCompAccelAngLimit_ = 1e3;

  // Target pose, velocity, and acceleration in the world frame
  sva::PTransformd targetPoseW_ = sva::PTransformd::Identity();
  sva::MotionVecd targetVelW_ = sva::MotionVecd::Zero();
  sva::MotionVecd targetAccelW_ = sva::MotionVecd::Zero();

  // Wrench in the surface frame
  sva::ForceVecd targetWrench_ = sva::ForceVecd::Zero();
  sva::ForceVecd measuredWrench_ = sva::ForceVecd::Zero();
  sva::ForceVecd filteredMeasuredWrench_ = sva::ForceVecd::Zero();

  mc_filter::LowPass<sva::ForceVecd> lowPass_;

  // Hold mode
  bool hold_ = false;

  void update(mc_solver::QPSolver & solver) override;

  void addToSolver(mc_solver::QPSolver & solver) override;
  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;
  void addToLogger(mc_rtc::Logger & logger) override;

private:
  /** Targets of SurfaceTransformTask should not be set by the user.
   *  Instead, the user can set the targetPose, targetVel, and targetAccel.
   *  Targets of SurfaceTransformTask are determined from the target values through the impedance equation.
   */
  using SurfaceTransformTask::refAccel;
  using SurfaceTransformTask::refVelB;
  using SurfaceTransformTask::target;
};

} // namespace force

} // namespace mc_tasks
