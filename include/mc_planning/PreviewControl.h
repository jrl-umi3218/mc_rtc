/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_planning/api.h>
#include <mc_rtc/log/Logger.h>

namespace mc_planning
{
/*! \brief Preview control for CoM trajectory generation.
 *
 *  Reference:
 *    Shuuji Kajita, et al., Biped walking pattern generation by using preview control of zero-moment point, ICRA 2003
 *    https://ieeexplore.ieee.org/abstract/document/1241826
 */
class MC_PLANNING_DLLAPI PreviewControl
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*! \brief Constructor.
   *
   *  \param name Name of the preview control
   */
  PreviewControl(const std::string & name = "");

  /*! \brief Reset the preview control.
   *
   *  \param comState CoM state (position, velocity, and acceleration) to reset
   *  \param zmp ZMP to reset
   */
  void reset(const Eigen::Vector3d & comState = Eigen::Vector3d::Zero(), double zmp = 0.0);

  /*! \brief Calculate the gain of the preview control
   *
   *  \param comHeight CoM height
   *  \param horizon Horizon of the preview control [sec]
   *  \param dt Sampling time of the preview control
   *
   *  \note If the CoM height, horizon, and dt do not change, this method only needs to be called once at the beginning.
   */
  void calcGain(double comHeight, double horizon, double dt);

  /*! \brief Set the reference ZMP trajectory. */
  void setRefZmpTraj(const Eigen::VectorXd & refZmpTraj);

  /*! \brief Update the preview control.
   *
   *  \note This method should be called once every control cycle. If you want to change the reference ZMP trajectory,
   * update it with setRefZmpTraj before calling this method.
   */
  void update();

  /*! \brief Get name. */
  const std::string & name() const noexcept
  {
    return name_;
  }

  /*! \brief Get the planned CoM state. */
  const Eigen::Vector3d & comState() const noexcept
  {
    return comState_;
  }

  /*! \brief Get the planned ZMP. */
  double zmp() const noexcept
  {
    return zmp_;
  }

  /*! \brief Get the size of preview window. */
  int previewSize() const
  {
    return static_cast<int>(f_.size());
  }

  /*! \brief Set the weight of the objective function.
   *
   *  \param Q Weight of output (ZMP) error
   *  \param R Weight of input (CoM jerk)
   */
  void setObjectiveWeight(double Q, double R)
  {
    Q_ = Q;
    R_ = R;
  }

  /*! \brief Add entries to the logger. */
  void addToLogger(mc_rtc::Logger & logger);

  /*! \brief Remove entries from the logger. */
  void removeFromLogger(mc_rtc::Logger & logger);

protected:
  std::string name_;

  Eigen::Vector3d comState_;
  double zmp_;

  /** Matrix of the linear state equation.
   *
   *  \f[
   *    x_{k+1} = A x_k + B u
   *    y_k = C x_k
   *  \f]
   *  where x is the [position, velocity, acceleration] of CoM, u is the CoM jerk, and y is the ZMP.
   *  @{
   */
  Eigen::Matrix3d A_;
  Eigen::Vector3d B_;
  Eigen::RowVector3d C_;
  /// @}

  // Output (ZMP) error weight
  double Q_ = 1.0;
  // Input (CoM jerk) weight
  double R_ = 1e-8;

  // Solution of the algebraic Riccati equation
  Eigen::Matrix3d P_;
  // Feedback gain
  Eigen::RowVector3d K_;
  // Preview gain
  Eigen::VectorXd f_;
  // Error of algebraic Riccati equation
  double riccatiError_ = 0;

  // Reference ZMP trajectory
  Eigen::VectorXd refZmpTraj_;
};
} // namespace mc_planning
