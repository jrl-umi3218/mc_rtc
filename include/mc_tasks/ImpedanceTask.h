/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_filter/LowPass.h>
#include <mc_filter/utils/clamp.h>
#include <mc_rtc/constants.h>
#include <mc_tasks/SurfaceTransformTask.h>

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
   * \param impM Impedance mass parameter
   *
   * \param impD Impedance damping parameter
   *
   * \param impK Impedance spring parameter
   *
   * \param ori_scale Scale of impedance parameters for orientation
   *
   * \throws If the body the task is attempting to control does not have a
   * sensor attached to it
   *
   */
  ImpedanceTask(const std::string & surfaceName,
                const mc_rbdyn::Robots & robots,
                unsigned robotIndex,
                double stiffness = 5.0,
                double weight = 1000.0,
                double impM = 10,
                double impD = 1000,
                double impK = 1000,
                double oriScale = 1);

  /*! \brief Reset the task
   *
   * Set the desired and compliance poses to the current surface, and reset the desired and compliance
   * velocity and acceleration to zero.
   *
   */
  void reset() override;

  /*! \brief Set the impedance parameters in the surface. */
  void impedance(const sva::ForceVecd & impM, const sva::ForceVecd & impD, const sva::ForceVecd & impK)
  {
    impedancePosition(impM.force(), impD.force(), impK.force());
    impedanceOrientation(impM.moment(), impD.moment(), impK.moment());
  }

  /*! \brief Set the translational impedance parameters in the surface. */
  void impedancePosition(const Eigen::Vector3d & impMPos,
                         const Eigen::Vector3d & impDPos,
                         const Eigen::Vector3d & impKPos)
  {
    impM_.force() = mc_filter::utils::clampAndWarn(impMPos, impMPosLower_, impMPosUpper_, "impedancePositionM");
    impD_.force() = mc_filter::utils::clampAndWarn(impDPos, impDPosLower_, impDPosUpper_, "impedancePositionD");
    impK_.force() = mc_filter::utils::clampAndWarn(impKPos, impKPosLower_, impKPosUpper_, "impedancePositionK");
  }

  /*! \brief Set the rotational impedance parameters in the surface. */
  void impedanceOrientation(const Eigen::Vector3d & impMOri,
                            const Eigen::Vector3d & impDOri,
                            const Eigen::Vector3d & impKOri)
  {
    impM_.moment() = mc_filter::utils::clampAndWarn(impMOri, impMOriLower_, impMOriUpper_, "impedanceOrientationM");
    impD_.moment() = mc_filter::utils::clampAndWarn(impDOri, impDOriLower_, impDOriUpper_, "impedanceOrientationD");
    impK_.moment() = mc_filter::utils::clampAndWarn(impKOri, impKOriLower_, impKOriUpper_, "impedanceOrientationK");
  }

  /*! \brief Get the impedance mass parameter represented in the surface. */
  const sva::ForceVecd & impedanceM() const noexcept
  {
    return impM_;
  }

  /*! \brief Set the impedance mass parameter represented in the surface. */
  void impedanceM(const sva::ForceVecd & impM)
  {
    impM_.force() = mc_filter::utils::clampAndWarn(impM.force(), impMPosLower_, impMPosUpper_, "impedancePositionM");
    impM_.moment() =
        mc_filter::utils::clampAndWarn(impM.moment(), impMOriLower_, impMOriUpper_, "impedanceOrientationM");
  }

  /*! \brief Get the impedance damper parameter represented in the surface. */
  const sva::ForceVecd & impedanceD() const noexcept
  {
    return impD_;
  }

  /*! \brief Set the impedance damper parameter represented in the surface. */
  void impedanceD(const sva::ForceVecd & impD)
  {
    impD_.force() = mc_filter::utils::clampAndWarn(impD.force(), impDPosLower_, impDPosUpper_, "impedancePositionD");
    impD_.moment() =
        mc_filter::utils::clampAndWarn(impD.moment(), impDOriLower_, impDOriUpper_, "impedanceOrientationD");
  }

  /*! \brief Get the impedance spring parameter represented in the surface. */
  const sva::ForceVecd & impedanceK() const noexcept
  {
    return impK_;
  }

  /*! \brief Set the impedance spring parameter represented in the surface. */
  void impedanceK(const sva::ForceVecd & impK)
  {
    impK_.force() = mc_filter::utils::clampAndWarn(impK.force(), impKPosLower_, impKPosUpper_, "impedancePositionK");
    impK_.moment() =
        mc_filter::utils::clampAndWarn(impK.moment(), impKOriLower_, impKOriUpper_, "impedanceOrientationK");
  }

  /*! \brief Get the wrench gain, which is multiplied by the wrench in the surface frame. */
  const sva::MotionVecd & wrenchGain() const noexcept
  {
    return wrenchGain_;
  }

  /*! \brief Set the wrench gain, which is multiplied by the wrench in the surface frame. */
  void wrenchGain(const sva::MotionVecd & gain)
  {
    wrenchGain_.linear() =
        mc_filter::utils::clampAndWarn(gain.linear(), wrenchGainLinLower_, wrenchGainLinUpper_, "wrenchGainLinear");
    wrenchGain_.angular() =
        mc_filter::utils::clampAndWarn(gain.angular(), wrenchGainAngLower_, wrenchGainAngUpper_, "wrenchGainAngular");
  }

  /*! \brief Get the desired pose of the surface in the world frame. */
  const sva::PTransformd & desiredPose() const noexcept
  {
    return desiredPoseW_;
  }

  /*! \brief Set the desired pose of the surface in the world frame. */
  void desiredPose(const sva::PTransformd & pose)
  {
    desiredPoseW_ = pose;
  }

  /*! \brief Get the desired velocity of the surface in the world frame. */
  const sva::MotionVecd & desiredVel() const noexcept
  {
    return desiredVelW_;
  }

  /*! \brief Set the desired velocity of the surface in the world frame. */
  void desiredVel(const sva::MotionVecd & vel)
  {
    desiredVelW_ = vel;
  }

  /*! \brief Get the desired acceleration of the surface in the world frame. */
  const sva::MotionVecd & desiredAccel() const noexcept
  {
    return desiredAccelW_;
  }

  /*! \brief Set the desired acceleration of the surface in the world frame. */
  void desiredAccel(const sva::MotionVecd & accel)
  {
    desiredAccelW_ = accel;
  }

  /*! \brief Get the relative pose from desired frame to compliance frame represented in the world frame. */
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
    sva::PTransformd T_0_d(Eigen::Matrix3d(desiredPoseW_.rotation()));
    return T_0_d * deltaCompPoseW_ * T_0_d.inv() * desiredPoseW_;
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
  double cutoffPeriod()
  {
    return lowPass_.cutoffPeriod();
  }

  /*! \brief Set the cutoff period for the low-pass filter of measured wrench. */
  void cutoffPeriod(double cutoffPeriod)
  {
    lowPass_.cutoffPeriod(cutoffPeriod);
  }

  /*! \brief Load parameters from a Configuration object. */
  void load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) override;

protected:
  // Impedance parameters represented in the surface frame
  sva::ForceVecd impM_; // must be set in the Constructor
  sva::ForceVecd impD_; // must be set in the Constructor
  sva::ForceVecd impK_; // must be set in the Constructor
  sva::MotionVecd wrenchGain_ = sva::MotionVecd::Zero();

  // Upper and lower values of impedance parameters
  const Eigen::Vector3d impMPosUpper_ = Eigen::Vector3d::Constant(1e6);
  const Eigen::Vector3d impMPosLower_ =
      Eigen::Vector3d::Constant(1e-6); // must be strictly positive to avoid zero division
  const Eigen::Vector3d impMOriUpper_ = Eigen::Vector3d::Constant(1e6);
  const Eigen::Vector3d impMOriLower_ =
      Eigen::Vector3d::Constant(1e-6); // must be strictly positive to avoid zero division
  const Eigen::Vector3d impDPosUpper_ = Eigen::Vector3d::Constant(1e6);
  const Eigen::Vector3d impDPosLower_ = Eigen::Vector3d::Constant(0);
  const Eigen::Vector3d impDOriUpper_ = Eigen::Vector3d::Constant(1e6);
  const Eigen::Vector3d impDOriLower_ = Eigen::Vector3d::Constant(0);
  const Eigen::Vector3d impKPosUpper_ = Eigen::Vector3d::Constant(1e6);
  const Eigen::Vector3d impKPosLower_ = Eigen::Vector3d::Constant(0);
  const Eigen::Vector3d impKOriUpper_ = Eigen::Vector3d::Constant(1e6);
  const Eigen::Vector3d impKOriLower_ = Eigen::Vector3d::Constant(0);
  const Eigen::Vector3d wrenchGainLinUpper_ = Eigen::Vector3d::Constant(1e6);
  const Eigen::Vector3d wrenchGainLinLower_ = Eigen::Vector3d::Constant(0);
  const Eigen::Vector3d wrenchGainAngUpper_ = Eigen::Vector3d::Constant(1e6);
  const Eigen::Vector3d wrenchGainAngLower_ = Eigen::Vector3d::Constant(0);

  /** Relative pose, velocity, and acceleration from desired frame to compliance frame represented in the world frame.
   *  To store these values across control cycles, represent them in a constant world frame instead of the time-varying
   *  surface frame.
   *  @{
   */
  sva::PTransformd deltaCompPoseW_ = sva::PTransformd::Identity();
  sva::MotionVecd deltaCompVelW_ = sva::MotionVecd::Zero();
  sva::MotionVecd deltaCompAccelW_ = sva::MotionVecd::Zero();
  /// @}

  // Limits of relative pose, velocity, and acceleration from desired frame to compliance frame.
  double deltaCompPoseLinLimit_ = 1.0;
  double deltaCompPoseAngLimit_ = mc_rtc::constants::PI;
  double deltaCompVelLinLimit_ = 1e3;
  double deltaCompVelAngLimit_ = 1e3;
  double deltaCompAccelLinLimit_ = 1e3;
  double deltaCompAccelAngLimit_ = 1e3;

  // Desired pose, velocity, and acceleration in the world frame
  sva::PTransformd desiredPoseW_ = sva::PTransformd::Identity();
  sva::MotionVecd desiredVelW_ = sva::MotionVecd::Zero();
  sva::MotionVecd desiredAccelW_ = sva::MotionVecd::Zero();

  // Wrench in the surface frame
  sva::ForceVecd targetWrench_ = sva::ForceVecd::Zero();
  sva::ForceVecd measuredWrench_ = sva::ForceVecd::Zero();
  sva::ForceVecd filteredMeasuredWrench_ = sva::ForceVecd::Zero();

  mc_filter::LowPass<sva::ForceVecd> lowPass_;

  void update(mc_solver::QPSolver & solver) override;

  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;
  void addToLogger(mc_rtc::Logger & logger) override;
  void removeFromLogger(mc_rtc::Logger & logger) override;

private:
  /** Targets of SurfaceTransformTask should not be set by the user.
   *  Instead, the user can set the desiredPose, desiredVel, and desiredAccel.
   *  Targets of SurfaceTransformTask are determined from the desired values through the impedance equation.
   */
  using SurfaceTransformTask::refAccel;
  using SurfaceTransformTask::refVelB;
  using SurfaceTransformTask::target;
};

} // namespace force

} // namespace mc_tasks
