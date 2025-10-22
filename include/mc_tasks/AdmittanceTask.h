/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_filter/utils/clamp.h>

#include <mc_tasks/TransformTask.h>

namespace mc_tasks
{

namespace force
{

/*! \brief Hybrid position-force control on a contacting end-effector.
 *
 * The AdmittanceTask is by default a TransformTask, i.e. pure position
 * control of a frame. Admittance coefficients that map force errors to
 * displacements (see [1] and [2]) are initially set to zero.
 *
 * When the admittance along one axis (Fx, Fy, Fz, Tx, Ty or Tz) is set to a
 * non-zero positive value, this axis switches from position to force control.
 * The goal is then to realize the prescribed target wrench at the control
 * frame (bis repetita placent: wrenches are expressed in the control frame of
 * the task, not in the sensor frame of the corresponding body). The force
 * control law applied is damping control [3].
 *
 * See the discussion in [4] for a comparison with the ComplianceTask.
 *
 * [1] https://en.wikipedia.org/wiki/Mechanical_impedance
 * [2] https://en.wikipedia.org/wiki/Impedance_analogy
 * [3] https://doi.org/10.1109/IROS.2010.5651082
 * [4] https://gite.lirmm.fr/multi-contact/mc_rtc/issues/34
 *
 */
struct MC_TASKS_DLLAPI AdmittanceTask : TransformTask
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*! \brief Initialize a new admittance task.
   *
   * \param frame Control frame
   *
   * \param stiffness Stiffness of the underlying Transform task
   *
   * \param weight Weight of the underlying Transform task
   *
   * \throws If the frame does not have a force sensor attached
   *
   */
  AdmittanceTask(const mc_rbdyn::RobotFrame & frame,
                 double stiffness = 5.0,
                 double weight = 1000.0,
                 bool showTarget = true,
                 bool showPose = true);

  /*! \brief Initialize a new admittance task.
   *
   * \param robotSurface Name of the surface frame to control, in which the
   * desired wrench will also be expressed
   *
   * \param robots Robots where the task will be applied
   *
   * \param robotIndex Which robot among the robots
   *
   * \param stiffness Stiffness of the underlying Transform task
   *
   * \param weight Weight of the underlying Transform task
   *
   * \throws If the body the task is attempting to control does not have a
   * sensor attached to it
   *
   */
  AdmittanceTask(const std::string & robotSurface,
                 const mc_rbdyn::Robots & robots,
                 unsigned robotIndex,
                 double stiffness = 5.0,
                 double weight = 1000.0,
                 bool showTarget = true,
                 bool showPose = true);

  /*! \brief Reset the task
   *
   * Set the end effector objective to the current position of the
   * end-effector, disable force control and reset admittance and target wrench
   * to zero.
   *
   */
  void reset() override;

  /*! \brief Get the admittance coefficients of the task
   *
   */
  const sva::ForceVecd & admittance() const { return admittance_; }

  /*! \brief Set the admittance coefficients of the task
   *
   * \param admittance Vector of positive admittance coefficients
   *
   */
  void admittance(const sva::ForceVecd & admittance) { admittance_ = admittance; }

  /*! \brief Get the current pose of the control frame in the inertial frame */
  sva::PTransformd surfacePose() const { return frame_->position(); }

  /*! \brief Get the target pose of the control frame in the world frame. */
  sva::PTransformd targetPose() { return this->target(); }

  /*! \brief Set target pose (position and orientation) of the frame in the world frame.
   *
   * \param X_0_target Plucker transform to the target frame.
   *
   */
  void targetPose(const sva::PTransformd & X_0_target) { this->target(X_0_target); }

  /*! \brief Transform from current frame pose to target. */
  sva::PTransformd poseError() { return targetPose() * surfacePose().inv(); }

  /*! \brief Get the target wrench in the control frame */
  const sva::ForceVecd & targetWrench() const { return targetWrench_; }

  /*! \brief Set the target wrench in world frame.
   *
   * \param wrench Target wrench in world frame
   */
  void targetWrenchW(const sva::ForceVecd & wrenchW) { targetWrench(frame_->position().dualMul(wrenchW)); }

  /*! \brief Set the target wrench in the control frame
   *
   * \param wrench Target wrench in the control frame
   *
   */
  void targetWrench(const sva::ForceVecd & wrench) { targetWrench_ = wrench; }

  /*! \brief Get the measured wrench in the control frame */
  sva::ForceVecd measuredWrench() const { return frame_->wrench(); }

  /*! \brief Set the maximum translation velocity of the task */
  void maxLinearVel(const Eigen::Vector3d & maxLinearVel)
  {
    if((maxLinearVel.array() <= 0.).any())
    {
      mc_rtc::log::error("discarding maxLinearVel update as it is not positive");
      ;
      return;
    }
    maxLinearVel_ = maxLinearVel;
  }

  /*! Get the maximum linear velocity of the task */
  const Eigen::Vector3d & maxLinearVel() const noexcept { return maxLinearVel_; }

  /*! \brief Set the maximum angular velocity of the task */
  void maxAngularVel(const Eigen::Vector3d & maxAngularVel)
  {
    if((maxAngularVel.array() <= 0.).any())
    {
      mc_rtc::log::error("discarding maxAngularVel update as it is not positive");
      ;
      return;
    }
    maxAngularVel_ = maxAngularVel;
  }

  /*! Get the maximum angular velocity of the task */
  const Eigen::Vector3d & maxAngularVel() const noexcept { return maxAngularVel_; }

  /*! \brief Set the gain of the low-pass filter on the reference velocity
   *
   * \param gain Gain between 0 and 1.
   */
  void velFilterGain(double gain) { velFilterGain_ = mc_filter::utils::clamp(gain, 0, 1); }

  /*! \brief Return the gain of the low-pass filter on the reference velocity */
  double velFilterGain() const noexcept { return velFilterGain_; }

  /*! \brief Set the reference body velocity.
   *
   * \param velB Feedforward body velocity
   *
   * Body velocity means that velB is the velocity of the control frame
   * expressed in the control frame itself. See e.g. (Murray et al., 1994, CRC
   * Press).
   *
   * \note The refVelB body velocity ultimately set as task target to the QP is
   * the sum of this feedforward velocity and a feedback term for force
   * control.
   *
   */
  void refVelB(const sva::MotionVecd & velB) { feedforwardVelB_ = velB; }

  /*! \brief Load parameters from a Configuration object */
  void load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) override;

protected:
  Eigen::Vector3d maxAngularVel_ = {0.1, 0.1, 0.1}; // [rad] / [s]
  Eigen::Vector3d maxLinearVel_ = {0.1, 0.1, 0.1}; // [m] / [s]
  double timestep_;
  double velFilterGain_ = 0.8; //< Gain for the low-pass filter on reference velocity [0..1]
  sva::ForceVecd admittance_ = sva::ForceVecd(Eigen::Vector6d::Zero());
  sva::ForceVecd targetWrench_ = sva::ForceVecd(Eigen::Vector6d::Zero());
  sva::ForceVecd wrenchError_ = sva::ForceVecd(Eigen::Vector6d::Zero());
  sva::MotionVecd feedforwardVelB_ = sva::MotionVecd(Eigen::Vector6d::Zero());
  sva::MotionVecd refVelB_ = sva::MotionVecd(Eigen::Vector6d::Zero());

  void update(mc_solver::QPSolver &) override;

  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;
  void addToLogger(mc_rtc::Logger & logger) override;

  bool showTarget_ = true;
  bool showPose_ = true;

  /** Transform's refVelB() becomes internal to the task. */
  using TransformTask::refVelB;

  /** Surface transform's target becomes internal to the task. Its setter is now targetPose(). */
  using TransformTask::target;

  /** Override addToSolver in order to get the timestep's solver automatically
   *
   */
  void addToSolver(mc_solver::QPSolver & solver) override;
};

} // namespace force

} // namespace mc_tasks
