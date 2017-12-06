#pragma once

#include <mc_tasks/AdmittanceTask.h>

namespace mc_tasks
{

/*! \brief Track center-of-pressure (CoP) references at contact
 *
 * ...
 *
 * [1] https://scaron.info/teaching/zero-tilting-moment-point.html
 *
 */
struct MC_TASKS_DLLAPI CoPTask: AdmittanceTask
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*! \brief Initialize a new CoP task.
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
  CoPTask(const std::string & robotSurface,
      const mc_rbdyn::Robots & robots,
      unsigned robotIndex,
      double timestep,
      double stiffness = 5.0, double weight = 1000.0);

  /*! \brief Reset the task
   *
   * Set the end effector objective to the current position of the
   * end-effector, disable CoP tracking and reset admittance and target CoP to
   * zero.
   *
   */
  virtual void reset();

  /*! \brief Set the target CoP in the surface frame
   *
   * \param targetCoP 2D vector of CoP coordinates in the surface frame
   *
   */
  void targetCoP(const Eigen::Vector2d & targetCoP)
  {
    targetCoP_ = targetCoP;
  }

  /*! \brief Get target CoP in the surface frame
   *
   */
  const Eigen::Vector2d & targetCoP() const
  {
    return targetCoP_;
  }

  /*! \brief Compute CoP in surface frame from sensor measurements
   *
   */
  const Eigen::Vector2d measuredCoP() const
  {
    const sva::ForceVecd w_surf = measuredWrench();
    const double Fz = w_surf.force()(2);
    if (Fz < 10.)  // [N]
    {
      return Eigen::Vector2d::Zero();
    }
    const Eigen::Vector3d tau_surf = w_surf.couple();
    return Eigen::Vector2d(-tau_surf(1) / Fz, +tau_surf(0) / Fz);
  }

  /*! \brief Set the target force in the surface frame
   *
   * \param targetForce 3D vector of target force in the surface frame
   *
   */
  void targetForce(const Eigen::Vector3d & targetForce)
  {
    targetForce_ = targetForce;
  }

  /*! \brief Get target force in the surface frame
   *
   */
  const Eigen::Vector3d & targetForce() const
  {
    return targetForce_;
  }

private:
  Eigen::Vector3d targetForce_ = Eigen::Vector3d::Zero();
  Eigen::Vector2d targetCoP_ = Eigen::Vector2d::Zero();

  void update() override;
};

}
