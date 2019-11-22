/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/DampingTask.h>

namespace mc_tasks
{

namespace force
{

/*! \brief Track center-of-pressure (CoP) references at contact
 *
 * The CoPTask is basically an AdmittanceTask where contact wrenches are
 * expressed in terms of center of pressure (a.k.a. ZMP, see e.g. [1]) rather
 * than torques. This is better-suited to locomotion applications.
 *
 * The CoP is well-defined only when the contact pressure is strictly positive.
 * When there is no contact pressure, the CoPTask will automatically disable
 * torque tracking in the underlying AdmittanceTask.
 *
 * [1] https://scaron.info/teaching/zero-tilting-moment-point.html
 *
 */
struct MC_TASKS_DLLAPI CoPTask : DampingTask
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
          double stiffness = 5.0,
          double weight = 1000.0);

  /*! \brief Reset the task
   *
   * Set the end effector objective to the current position of the
   * end-effector, disable CoP tracking and reset admittance and target CoP to
   * zero.
   *
   */
  void reset() override;

  /** Add support for the following entries
   *
   * - copError Threshold for (targetCoP - measureCoP).norm()
   * - force Force threshold, similar to wrench for AdmittanceTask
   *
   */
  std::function<bool(const mc_tasks::MetaTask &, std::string &)> buildCompletionCriteria(
      double dt,
      const mc_rtc::Configuration & config) const override;

  /*! \brief Measured CoP in surface frame.
   *
   */
  Eigen::Vector2d measuredCoP() const
  {
    return robots_.robot(rIndex_).cop(surface_.name());
  }

  /*! \brief Measured CoP in world frame.
   *
   */
  Eigen::Vector3d measuredCoPW() const
  {
    return robots_.robot(rIndex_).copW(surface_.name());
  }

  /*! \brief Set targent wrench to zero.
   *
   */
  void setZeroTargetWrench()
  {
    AdmittanceTask::targetWrench(sva::ForceVecd{Eigen::Vector6d::Zero()});
    targetCoP(Eigen::Vector2d::Zero());
  }

  /*! \brief Get target CoP in the surface frame.
   *
   */
  const Eigen::Vector2d & targetCoP() const
  {
    return targetCoP_;
  }

  /*! \brief Get target CoP in the world frame.
   *
   */
  Eigen::Vector3d targetCoPW() const
  {
    Eigen::Vector3d cop_s;
    cop_s << targetCoP_, 0.;
    sva::PTransformd X_0_s = robots_.robot(rIndex_).surfacePose(surface_.name());
    return X_0_s.translation() + X_0_s.rotation().transpose() * cop_s;
  }

  /*! \brief Set target CoP in the surface frame.
   *
   * \param targetCoP 2D vector of CoP coordinates in the surface frame
   *
   */
  void targetCoP(const Eigen::Vector2d & targetCoP)
  {
    targetCoP_ = targetCoP;
  }

  /*! \brief Get target force in the surface frame
   *
   */
  const Eigen::Vector3d & targetForce() const
  {
    return targetForce_;
  }

  /*! \brief Set target force in the surface frame
   *
   * \param targetForce 3D vector of target force in the surface frame
   *
   */
  void targetForce(const Eigen::Vector3d & targetForce)
  {
    targetForce_ = targetForce;
  }

  /*! \brief Set target force in the world frame
   *
   * \param targetForce 3D vector of target force in the world frame
   */
  void targetForceW(const Eigen::Vector3d & targetForceW)
  {
    const auto & X_0_rh = robots_.robot(rIndex_).surface(surface_.name()).X_0_s(robots_.robot(rIndex_));
    targetForce(X_0_rh.dualMul(sva::ForceVecd(Eigen::Vector3d::Zero(), targetForceW)).force());
  }

  /*! \brief Get target wrench in the surface frame
   *
   */
  const sva::ForceVecd & targetWrench() const
  {
    return AdmittanceTask::targetWrench();
  }

  void addToLogger(mc_rtc::Logger & logger) override;
  void removeFromLogger(mc_rtc::Logger & logger) override;
  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;

private:
  Eigen::Vector2d targetCoP_ = Eigen::Vector2d::Zero();
  Eigen::Vector3d targetForce_ = Eigen::Vector3d::Zero();

  void update() override;

  using AdmittanceTask::targetWrench;
  using AdmittanceTask::targetWrenchW;
};

} // namespace force

} // namespace mc_tasks
