#pragma once

#include <mc_tasks/SurfaceTransformTask.h>

namespace mc_tasks
{

/*! \brief Hybrid position-force control on a contacting end-effector.
 *
 * The AdmittanceTask is by default a SurfaceTransformTask, i.e. pure position
 * control of a surface frame. Admittance coefficients that map force errors to
 * displacements (see [1] and [2]) are initially set to zero. 
 *
 * When the admittance along one axis (Fx, Fy, Fz, Tx, Ty or Tz) is set to a
 * non-zero positive value, this axis switches from position to force control.
 * The goal is then to realize the prescribed target wrench at the surface
 * frame (bis repetita placent: wrenches are expressed in the surface frame of
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
struct MC_TASKS_DLLAPI AdmittanceTask: SurfaceTransformTask
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*! \brief Initialize a new admittance task.
   *
   * \param robotSurface Name of the surface frame to control, in which the
   * desired wrench will also be expressed
   *
   * \param robots Robots where the task will be applied
   *
   * \param robotIndex Which robot among the robots
   *
   * \param timestep Timestep of the controller
   *
   * \param stiffness Stiffness of the underlying SurfaceTransform task
   *
   * \param weight Weight of the underlying SurfaceTransform task
   *
   * \throws If the body the task is attempting to control does not have a
   * sensor attached to it
   *
   */
  AdmittanceTask(const std::string & robotSurface,
      const mc_rbdyn::Robots & robots,
      unsigned robotIndex,
      double timestep,
      double stiffness = 5.0, double weight = 1000.0);

  /*! \brief Reset the task
   *
   * Set the end effector objective to the current position of the
   * end-effector, disable force control and reset admittance and target wrench
   * to zero.
   *
   */
  virtual void reset() override;

  /*! \brief Get the admittance coefficients of the task
   *
   */
  const sva::ForceVecd & admittance() const
  {
    return admittance_;
  }

  /*! \brief Set the admittance coefficients of the task
   *
   * \param admittance Vector of positive admittance coefficients
   *
   */
  void admittance(const sva::ForceVecd & admittance)
  {
    admittance_ = admittance;
  }

  /*! \brief Set the task stiffness and damping
   *
   * Damping is set to the critical value of 2 * sqrt(stiffness).
   *
   * \param stiffness Task stiffness
   *
   */
  void setCriticalGains(double stiffness)
  {
    setGains(stiffness, 2 * std::sqrt(stiffness));
  }

  /*! \brief Get the current pose of the robot surface in the inertial frame
   *
   */
  sva::PTransformd surfacePose() const
  {
    return robot_.surface(surfaceName).X_0_s(robot_);
  }

  /*! \brief Get the target pose for the position task
   *
   */
  sva::PTransformd targetPose()
  {
    return this->target();
  }

  /*! \brief Set target position and orientation
   *
   * \param X_0_target Plucker transform from the world frame to the target frame.
   *
   */
  void targetPose(const sva::PTransformd & X_0_target)
  {
    this->target(X_0_target);
  }

  /*! \brief Get the target wrench in the surface frame
   *
   */
  const sva::ForceVecd & targetWrench() const
  {
    return targetWrench_;
  }

  /*! \brief Set the target wrench in the surface frame
   *
   * \param wrench Target wrench in the surface frame
   *
   */
  void targetWrench(const sva::ForceVecd& wrench)
  {
    targetWrench_ = wrench;
  }

  /*! \brief Get the measured wrench in the surface frame
   *
   */
  sva::ForceVecd measuredWrench() const
  {
    sva::ForceVecd w_fsactual = sensor_.removeGravity(robot_);
    return X_fsactual_surf_.dualMul(w_fsactual);
  }

  /*! \brief Set the maximum translation velocity of the task */
  void maxTransVel(const Eigen::Vector3d & maxTransVel)
  {
    if ((maxTransVel.array() <= 0.).any())
    {
      LOG_ERROR("discarding maxTransVel update as it is not positive");
      return;
    }
    maxTransVel_ = maxTransVel;
  }

  /*! \brief Set the maximum angular velocity of the task */
  void maxRpyVel(const Eigen::Vector3d & maxRpyVel)
  {
    if ((maxRpyVel.array() <= 0.).any())
    {
      LOG_ERROR("discarding maxRpyVel update as it is not positive");
      return;
    }
    maxRpyVel_ = maxRpyVel;
  }

  /*! \brief Get the current task stiffness */
  double stiffness()
  {
    return SurfaceTransformTask::stiffness();
  }

protected:
  Eigen::Vector3d maxAngularVel_ = Eigen::Vector3d(0.1, 0.1, 0.1);  // [rad] / [s]
  Eigen::Vector3d maxLinearVel_ = Eigen::Vector3d(0.1, 0.1, 0.1);  // [m] / [s]
  const mc_rbdyn::Robot & robot_;
  const mc_rbdyn::Surface & surface_;
  const mc_rbdyn::ForceSensor & sensor_;
  const sva::PTransformd X_fsactual_surf_;
  double timestep_;
  std::map<char, bool> isClampingAngularVel_ = {{'x', false}, {'y', false}, {'z', false}};
  std::map<char, bool> isClampingLinearVel_ = {{'x', false}, {'y', false}, {'z', false}};
  sva::ForceVecd admittance_ = sva::ForceVecd(Eigen::Vector6d::Zero());
  sva::ForceVecd targetWrench_;
  sva::ForceVecd wrenchError_;
  sva::MotionVecd refVel_;

  void update() override;

  void addToLogger(mc_rtc::Logger & logger) override;

  void removeFromLogger(mc_rtc::Logger & logger) override;

  /** Don't use surface transform's stiffness() setter as it applies critical
   * damping, which is usually not good for admittance control. Use
   * setCriticalGains() if you do desire this behavior.
   *
   */
  using SurfaceTransformTask::stiffness;

  /** Surface transform's target becomes internal to the task. Its setter is
   * now targetPose().
   *
   */
  using SurfaceTransformTask::target;
};

}
