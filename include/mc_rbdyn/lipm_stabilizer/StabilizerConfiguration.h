/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/Configuration.h>
#include <mc_rtc/logging.h>

namespace mc_rbdyn
{

namespace lipm_stabilizer
{
/** Foot sole properties.
 *
 */
struct Sole
{
  double friction = 0.7;
  double halfLength = 0.112; // [m]
  double halfWidth = 0.065; // [m]
};

/** Weights for force distribution quadratic program (FDQP).
 *
 */
struct FDQPWeights
{
  double ankleTorqueSqrt;
  double netWrenchSqrt;
  double pressureSqrt;
};
} // namespace lipm_stabilizer
} // namespace mc_rbdyn

namespace mc_rtc
{
using FDQPWeights = mc_rbdyn::lipm_stabilizer::FDQPWeights;
using Sole = mc_rbdyn::lipm_stabilizer::Sole;
/**
 * @brief Load Sole properties
 */
template<>
struct ConfigurationLoader<Sole>
{
  static Sole load(const mc_rtc::Configuration & config)
  {
    Sole sole;
    config("friction", sole.friction);
    config("half_length", sole.halfLength);
    config("half_width", sole.halfWidth);
    return sole;
  }

  static mc_rtc::Configuration save(const Sole & sole)
  {
    mc_rtc::Configuration config;
    config.add("friction", sole.friction);
    config.add("half_length", sole.halfLength);
    config.add("half_width", sole.halfWidth);
    return config;
  }
};

/**
 * @brief Read force distribution QP weights from configuration.
 */
template<>
struct ConfigurationLoader<FDQPWeights>
{
  static FDQPWeights load(const mc_rtc::Configuration & config)
  {
    FDQPWeights weights;
    double ankleTorqueWeight = config("ankle_torque");
    double netWrenchWeight = config("net_wrench");
    double pressureWeight = config("pressure");
    weights.ankleTorqueSqrt = std::sqrt(ankleTorqueWeight);
    weights.netWrenchSqrt = std::sqrt(netWrenchWeight);
    weights.pressureSqrt = std::sqrt(pressureWeight);
    return weights;
  }

  static mc_rtc::Configuration save(const FDQPWeights & weights)
  {
    mc_rtc::Configuration config;
    config.add("ankle_torque", std::pow(weights.ankleTorqueSqrt, 2));
    config.add("net_wrench", std::pow(weights.netWrenchSqrt, 2));
    config.add("pressure", std::pow(weights.pressureSqrt, 2));
    return config;
  }
};
} // namespace mc_rtc

namespace mc_rbdyn
{
namespace lipm_stabilizer
{

struct StabilizerConfiguration
{
  FDQPWeights fdqpWeights;
  Sole sole;
  Eigen::Vector2d comAdmittance = Eigen::Vector2d::Zero(); /**< Admittance gains for CoM admittance control */
  Eigen::Vector2d copAdmittance = Eigen::Vector2d::Zero(); /**< Admittance gains for foot damping control */
  double dfzAdmittance = 1e-4; /**< Admittance for foot force difference control */
  double dfzDamping = 0.; /**< Damping term in foot force difference control */
  double dcmPropGain = 1.; /**< Proportional gain on DCM error */
  double dcmIntegralGain = 5.; /**< Integral gain on DCM error */
  double dcmDerivGain = 0.; /**< Derivative gain on DCM error */
  double dcmIntegratorTimeConstant = 15.;
  double dcmDerivatorTimeConstant = 1.;
  std::vector<std::string> comActiveJoints; /**< Joints used by CoM IK task */
  Eigen::Vector3d comStiffness = {1000., 1000., 100.}; /**< Stiffness of CoM IK task */
  double comWeight = 1000.; /**< Weight of CoM IK task */
  double comHeight = 0.84; /**< Desired height of the CoM */
  double maxCoMHeight = 0.92; /**< Maximum height of the CoM */
  double minCoMHeight = 0.6; /**< Minimum height of the CoM */
  sva::MotionVecd contactDamping = sva::MotionVecd::Zero();
  sva::MotionVecd contactStiffness = sva::MotionVecd::Zero();
  double contactWeight = 100000.; /**< Weight of contact IK tasks */
  double swingFootStiffness = 2000.; /**< Stiffness of swing foot IK task */
  double swingFootWeight = 500.; /**< Weight of swing foot IK task */
  double vdcFrequency = 1.; /**< Frequency used in double-support vertical drift compensation */
  double vdcStiffness = 1000.; /**< Stiffness used in single-support vertical drift compensation */
  double zmpccIntegratorLeakRate = 0.1;

  void load(const mc_rtc::Configuration & config)
  {
    fdqpWeights = config("fdqp_weights");

    if(config.has("admittance"))
    {
      auto admittance = config("admittance");
      admittance("com", comAdmittance);
      admittance("cop", copAdmittance);
      admittance("dfz", dfzAdmittance);
      admittance("dfz_damping", dfzDamping);
    }
    if(config.has("dcm_tracking"))
    {
      auto dcmTracking = config("dcm_tracking");
      if(dcmTracking.has("gains"))
      {
        dcmTracking("gains")("prop", dcmPropGain);
        dcmTracking("gains")("integral", dcmIntegralGain);
        dcmTracking("gains")("deriv", dcmDerivGain);
      }
      dcmTracking("derivator_time_constant", dcmDerivatorTimeConstant);
      dcmTracking("integrator_time_constant", dcmIntegratorTimeConstant);
    }
    if(config.has("tasks"))
    {
      auto tasks = config("tasks");
      if(tasks.has("com"))
      {
        tasks("com")("active_joints", comActiveJoints);
        tasks("com")("stiffness", comStiffness);
        tasks("com")("weight", comWeight);
        tasks("com")("height", comHeight);
        tasks("com")("max_height", maxCoMHeight);
        tasks("com")("min_height", minCoMHeight);
      }
      if(tasks.has("contact"))
      {
        double d = tasks("contact")("damping");
        double k = tasks("contact")("stiffness");
        contactDamping = sva::MotionVecd({d, d, d}, {d, d, d});
        contactStiffness = sva::MotionVecd({k, k, k}, {k, k, k});
        tasks("contact")("stiffness", contactStiffness);
        tasks("contact")("weight", contactWeight);
      }
      if(tasks.has("swing_foot"))
      {
        tasks("swing_foot")("stiffness", swingFootStiffness);
        tasks("swing_foot")("weight", swingFootWeight);
      }
    }
    if(config.has("vdc"))
    {
      auto vdc = config("vdc");
      vdc("frequency", vdcFrequency);
      vdc("stiffness", vdcStiffness);
    }
    if(config.has("zmpcc"))
    {
      auto zmpcc = config("zmpcc");
      zmpcc("integrator_leak_rate", zmpccIntegratorLeakRate);
    }

    config("sole", sole);
  }
};
} // namespace lipm_stabilizer
} // namespace mc_rbdyn

namespace mc_rtc
{
using StabilizerConfiguration = mc_rbdyn::lipm_stabilizer::StabilizerConfiguration;

template<>
struct ConfigurationLoader<StabilizerConfiguration>
{
  static StabilizerConfiguration load(const mc_rtc::Configuration & config)
  {
    StabilizerConfiguration stabi;
    stabi.load(config);
    return stabi;
  }

  static mc_rtc::Configuration save(const StabilizerConfiguration &)
  {
    LOG_ERROR("Attempting to save the lipm stabilizer configuration but this feature has not been implemented yet.");
    return mc_rtc::Configuration{};
  }
};
} // namespace mc_rtc
