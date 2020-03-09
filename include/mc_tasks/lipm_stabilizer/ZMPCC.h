/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once
#include <mc_filter/LeakyIntegrator.h>
#include <mc_rbdyn/lipm_stabilizer/ZMPCCConfiguration.h>
#include <SpaceVecAlg/SpaceVecAlg>

namespace mc_rtc
{
struct Logger;
namespace gui
{
struct StateBuilder;
} // namespace gui
} // namespace mc_rtc

namespace mc_tasks
{
namespace lipm_stabilizer
{
/** ZMP Compensation Control (ZMPCC)
 *
 * CoM admittance based on ZMP Compensation Control.
 * This approach is based on Section 6.2.2 of Dr Nagasaka's PhD thesis
 * "体幹位置コンプライアンス制御によるモデル誤差吸収" (1999) from
 * <https://sites.google.com/site/humanoidchannel/home/publication>.
 */
struct ZMPCC
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using ZMPCCConfiguration = mc_rbdyn::lipm_stabilizer::ZMPCCConfiguration;

  /** Resets everything to zero */
  void reset();

  /** Enable/Disable ZMPCC
   *
   * \param state Whether zmpcc is enabled
   *   - true: compute ZMPCC compensation
   *   - false: ZMPCC compensation gains are set to zero, and the integrator
   *   gradually leaks to zero
   */
  void enabled(bool state)
  {
    enabled_ = state;
  }

  /** Returns whether ZMPCC compensation is enabled
   */
  bool enabled() const
  {
    return enabled_;
  }

  /** Configures ZMPCC parameters
   *
   * \param config ZMPCC parameters
   *
   * \note this configuration may be modified interactively by the GUI elements
   */
  void configure(ZMPCCConfiguration & config)
  {
    config_ = config;
    integrator_.saturation(config.maxCoMOffset);
    integrator_.rate(config_.integratorLeakRate);
    mc_filter::utils::clampInPlaceAndWarn(config_.comAdmittance.x(), 0., config_.maxCoMAdmitance, "CoM x-admittance");
    mc_filter::utils::clampInPlaceAndWarn(config_.comAdmittance.y(), 0., config_.maxCoMAdmitance, "CoM y-admittance");
  }

  /** Returns ZMPCC configuration */
  const ZMPCCConfiguration & config() const
  {
    return config_;
  }

  /** Compute CoM offset due to ZMPCC compensation
   *
   * \param distribZMP Distributed ZMP target
   * \param measuredZMP Measured ZMP
   * \param zmpFrame Frame in which the ZMP is computed
   * \param dt Control timestep
   */
  void update(const Eigen::Vector3d & distribZMP,
              const Eigen::Vector3d & measuredZMP,
              const sva::PTransformd & zmpFrame,
              double dt);

  /**
   * \brief Apply ZMPCC in-place to the com
   *
   * \param com Desired CoM position
   * \param comd Desired CoM velocity
   * \param comdd Desired CoM acceleration
   */
  void apply(Eigen::Vector3d & com, Eigen::Vector3d & comd, Eigen::Vector3d & comdd);

  void addToGUI(mc_rtc::gui::StateBuilder & gui, const std::vector<std::string> & category);
  void removeFromGUI(mc_rtc::gui::StateBuilder & gui, const std::vector<std::string> & category);
  void addToLogger(mc_rtc::Logger & logger, const std::string & name);
  void removeFromLogger(mc_rtc::Logger & logger, const std::string & name);

protected:
  ZMPCCConfiguration config_; /**< Configuration of ZMPCC gains and parameters */
  Eigen::Vector3d comAccel_ = Eigen::Vector3d::Zero(); /**< Additional CoM acceleration from ZMPCC */
  Eigen::Vector3d comOffset_ = Eigen::Vector3d::Zero(); /**< Additional CoM position offset from ZMPCC */
  Eigen::Vector3d comVel_ = Eigen::Vector3d::Zero(); /**< Additional CoM velocity from ZMPCC */
  Eigen::Vector3d error_ = Eigen::Vector3d::Zero(); /**< Error between distributed and measured ZMP */
  mc_filter::LeakyIntegrator<Eigen::Vector3d> integrator_; /**< Leaky integrator for the CoM offset added by ZMPCC */
  bool enabled_ = true;
};
} // namespace lipm_stabilizer
} // namespace mc_tasks
