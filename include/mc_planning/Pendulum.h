/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is inspired by Stephane's Caron original implementation as part of
 * lipm_walking_controller <https://github.com/stephane-caron/lipm_walking_controller>
 */

#pragma once
#include <mc_planning/api.h>
#include <SpaceVecAlg/SpaceVecAlg>

namespace mc_planning
{
/** State of the inverted pendulum model.
 *
 */
struct MC_PLANNING_DLLAPI Pendulum
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Creates an empty pendulum. Call reset
   *
   * Call reset(double, const Eigen::Vector3d &, const Eigen::Vector3d &, const Eigen::Vector3d &) before use
   */
  Pendulum();

  /** Initialize state from CoM position and its derivatives.
   *
   * \param lambda Pendulum constant. lambda = gravity/(height above ground)
   * \param com CoM position.
   * \param comd CoM velocity.
   * \param comdd CoM acceleration.
   */
  Pendulum(double lamda,
           const Eigen::Vector3d & com = Eigen::Vector3d::Zero(),
           const Eigen::Vector3d & comd = Eigen::Vector3d::Zero(),
           const Eigen::Vector3d & comdd = Eigen::Vector3d::Zero());

  /** Complete inverted pendulum inputs (ZMP and natural frequency) from contact plane.
   *
   * \param p,n Contact plane in which the ZMP is considered.
   *
   * \note The current CoM position and acceleration are used to compute the
   * ZMP in the desired plane.
   */
  void completeIPM(const Eigen::Vector3d & p, const Eigen::Vector3d & n);

  /** Integrate constant CoM jerk for a given duration.
   *
   * \param comddd CoM jerk.
   *
   * \param dt Integration step.
   */
  void integrateCoMJerk(const Eigen::Vector3d & comddd, double dt);

  /** Integrate in floating-base inverted pendulum mode with constant inputs.
   *
   * \param zmp Zero-tilting Moment Point, i.e. net force application point.
   *
   * \param lambda Normalized stiffness of the pendulum.
   *
   * \param dt Duration of integration step.
   */
  void integrateIPM(Eigen::Vector3d zmp, double lambda, double dt);

  /** Reset to a new state from CoM position and its derivatives.
   *
   * \param lambda Pendulum constant. lambda = gravity/(height above ground)
   * \param com CoM position.
   * \param comd CoM velocity.
   * \param comdd CoM acceleration.
   */
  void reset(double lambda,
             const Eigen::Vector3d & com,
             const Eigen::Vector3d & comd = Eigen::Vector3d::Zero(),
             const Eigen::Vector3d & comdd = Eigen::Vector3d::Zero());

  /** CoM position in the world frame. */
  const Eigen::Vector3d & com() const
  {
    return com_;
  }

  /** CoM velocity in the world frame. */
  const Eigen::Vector3d & comd() const
  {
    return comd_;
  }

  /** CoM acceleration in the world frame. */
  const Eigen::Vector3d & comdd() const
  {
    return comdd_;
  }

  /** Divergent component of motion. */
  Eigen::Vector3d dcm() const
  {
    return com_ + comd_ / omega_;
  }

  /** Natural frequency. */
  double omega() const
  {
    return omega_;
  }

  /** Zero-tilting moment point.
   *
   * \note In the linear inverted pendulum mode, the ZMP coincides with the
   * centroidal moment pivot (CMP) or its extended version (eCMP).
   */
  const Eigen::Vector3d & zmp() const
  {
    return zmp_;
  }

  /** Velocity of the zero-tilting moment point.
   */
  const Eigen::Vector3d & zmpd() const
  {
    return zmpd_;
  }

protected:
  Eigen::Vector3d com_ = Eigen::Vector3d::Zero(); /**< Position of the center of mass */
  Eigen::Vector3d comd_ = Eigen::Vector3d::Zero(); /**< Velocity of the center of mass */
  Eigen::Vector3d comdd_ = Eigen::Vector3d::Zero(); /**< Acceleration of the center of mass */
  Eigen::Vector3d comddd_ = Eigen::Vector3d::Zero(); /**< Jerk of the center of mass */
  Eigen::Vector3d zmp_ = Eigen::Vector3d::Zero(); /**< Position of the zero-tilting moment point */
  Eigen::Vector3d zmpd_ = Eigen::Vector3d::Zero(); /**< Velocity of the zero-tilting moment point */
  double omega_ = 0; /**< Natural frequency in [Hz] */
};
} // namespace mc_planning
