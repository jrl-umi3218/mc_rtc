/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is inspired by Stephane's Caron original implementation as part of
 * lipm_walking_controller <https://github.com/stephane-caron/lipm_walking_controller>
 */

#pragma once
#include <mc_planning/api.h>
#include <mc_rbdyn/lipm_stabilizer/Contact.h>

namespace mc_planning
{
/** State of the inverted pendulum model.
 *
 */
struct MC_PLANNING_DLLAPI Pendulum
{
  using Contact = mc_rbdyn::lipm_stabilizer::Contact;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** Initialize state from CoM position and its derivatives.
   *
   * \param com CoM position.
   *
   * \param comd CoM velocity.
   *
   * \param comdd CoM acceleration.
   */
  Pendulum(const Eigen::Vector3d & com = Eigen::Vector3d::Zero(),
           const Eigen::Vector3d & comd = Eigen::Vector3d::Zero(),
           const Eigen::Vector3d & comdd = Eigen::Vector3d::Zero());

  /** Complete inverted pendulum inputs (ZMP and natural frequency) from contact plane.
   *
   * \param plane Contact plane in which the ZMP is considered.
   *
   * \note The current CoM position and acceleration are used to compute the
   * ZMP in the desired plane.
   */
  void completeIPM(const Contact & plane);

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
   * \param com CoM position.
   *
   * \param comd CoM velocity.
   *
   * \param comdd CoM acceleration.
   */
  void reset(const Eigen::Vector3d & com,
             const Eigen::Vector3d & comd = Eigen::Vector3d::Zero(),
             const Eigen::Vector3d & comdd = Eigen::Vector3d::Zero());

  /** Reset CoM height above a given contact plane.
   *
   * \param height CoM height above contact plane.
   *
   * \param contact Contact plane.
   */
  void resetCoMHeight(double height, const Contact & contact);

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
  Eigen::Vector3d com_; /**< Position of the center of mass */
  Eigen::Vector3d comd_; /**< Velocity of the center of mass */
  Eigen::Vector3d comdd_; /**< Acceleration of the center of mass */
  Eigen::Vector3d comddd_; /**< Jerk of the center of mass */
  Eigen::Vector3d zmp_; /**< Position of the zero-tilting moment point */
  Eigen::Vector3d zmpd_; /**< Velocity of the zero-tilting moment point */
  double omega_; /**< Natural frequency in [Hz] */
};
} // namespace mc_planning
