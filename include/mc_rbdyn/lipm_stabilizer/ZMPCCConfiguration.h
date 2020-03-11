/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once
#include <mc_rbdyn/api.h>
#include <mc_rtc/Configuration.h>
#include <Eigen/Core>

namespace mc_rbdyn
{
namespace lipm_stabilizer
{
struct MC_RBDYN_DLLAPI ZMPCCConfiguration
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector2d comAdmittance = Eigen::Vector2d::Zero(); /**< Admittance gains for CoM admittance control */
  double integratorLeakRate = 0.1; /**< Leak rate */
  double maxCoMOffset = 0.05; /**< Maximum CoM offset due to admittance control in [m] */
  double maxCoMAdmitance = 20; /**< Maximum admittance for CoM admittance control */
};
} // namespace lipm_stabilizer
} // namespace mc_rbdyn

namespace mc_rtc
{
/**
 * @brief Read force distribution QP weights from configuration.
 */
template<>
struct ConfigurationLoader<mc_rbdyn::lipm_stabilizer::ZMPCCConfiguration>
{
  static mc_rbdyn::lipm_stabilizer::ZMPCCConfiguration load(const mc_rtc::Configuration & config)
  {
    mc_rbdyn::lipm_stabilizer::ZMPCCConfiguration zmpcc;
    if(config.has("zmpcc"))
    {
      config("zmpcc")("comAdmittance", zmpcc.comAdmittance);
      config("zmpcc")("maxCoMOffset", zmpcc.maxCoMOffset);
      config("zmpcc")("integratorLeakRate", zmpcc.integratorLeakRate);
    }
    return zmpcc;
  }

  static mc_rtc::Configuration save(const mc_rbdyn::lipm_stabilizer::ZMPCCConfiguration & zmpcc)
  {
    mc_rtc::Configuration config;
    config.add("comAdmittance", zmpcc.comAdmittance);
    config.add("maxCoMOffset", zmpcc.maxCoMOffset);
    config.add("integratorLeakRate", zmpcc.integratorLeakRate);
    return config;
  }
};
} // namespace mc_rtc
