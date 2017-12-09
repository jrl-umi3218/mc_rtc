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
  virtual void reset();

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

  /*! \brief Get target translation and orientation for the position task
   *
   */
  const sva::PTransformd & targetPose() const
  {
    return X_0_target_;
  }

  /*! \brief Set target position and orientation
   *
   * \param X_0_target Plucker transform from the world frame to the target frame.
   *
   */
  void targetPose(const sva::PTransformd & X_0_target)
  {
    X_0_target_ = X_0_target;
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

  /*! \brief Set the maximum translation of the task */
  void maxTransPos(const Eigen::Vector3d & maxTransPos)
  {
    if ((maxTransPos.array() <= 0.).any())
    {
      LOG_ERROR("discarding maxTransPos update as it is not positive");
      return;
    }
    maxTransPos_ = maxTransPos;
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

  /*! \brief Set the maximum angular position of the task */
  void maxRpyPos(const Eigen::Vector3d & maxRpyPos)
  {
    if ((maxRpyPos.array() <= 0.).any())
    {
      LOG_ERROR("discarding maxRpyPos update as it is not positive");
      return;
    }
    maxRpyPos_ = maxRpyPos;
  }

protected:
  const mc_rbdyn::Surface & surface_;
  sva::ForceVecd admittance_ = sva::ForceVecd(Eigen::Vector6d::Zero());
  const mc_rbdyn::Robot & robot_;

  void update() override;

private:
  sva::ForceVecd wrenchError_ = sva::ForceVecd(Eigen::Vector6d::Zero());
  sva::PTransformd X_0_target_;
  sva::ForceVecd targetWrench_ = sva::ForceVecd(Eigen::Vector6d::Zero());
  const mc_rbdyn::ForceSensor & sensor_;
  double timestep_;
  Eigen::Vector3d trans_target_delta_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d rpy_target_delta_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d maxTransPos_ = Eigen::Vector3d(0.1, 0.1, 0.1);  // [m]
  Eigen::Vector3d maxTransVel_ = Eigen::Vector3d(0.1, 0.1, 0.1);  // [m] / [s]
  Eigen::Vector3d maxRpyPos_ = Eigen::Vector3d(0.5, 0.5, 0.5);  // [rad]
  Eigen::Vector3d maxRpyVel_ = Eigen::Vector3d(0.1, 0.1, 0.1);  // [rad] / [s]
  const sva::PTransformd X_fsactual_surf_;

  void addToLogger(mc_rtc::Logger & logger) override;
  void removeFromLogger(mc_rtc::Logger & logger) override;
};

}
