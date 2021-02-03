/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once
#include <mc_filter/utils/clamp.h>
#include <mc_rbdyn/api.h>
#include <mc_rbdyn/lipm_stabilizer/ZMPCCConfiguration.h>
#include <mc_rtc/Configuration.h>
#include <mc_rtc/logging.h>

namespace mc_rbdyn
{
namespace lipm_stabilizer
{

/** Weights for force distribution quadratic program (FDQP). */
struct MC_RBDYN_DLLAPI FDQPWeights
{
  FDQPWeights() : ankleTorqueSqrt(std::sqrt(100)), netWrenchSqrt(std::sqrt(10000)), pressureSqrt(std::sqrt(1)) {}
  /* All weights must be strictly positive */
  FDQPWeights(double netWrench, double ankleTorque, double pressure)
  : ankleTorqueSqrt(std::sqrt(ankleTorque)), netWrenchSqrt(std::sqrt(netWrench)), pressureSqrt(std::sqrt(pressure))
  {
  }
  double ankleTorqueSqrt;
  double netWrenchSqrt;
  double pressureSqrt;

  void load(const mc_rtc::Configuration & config)
  {
    if(config.has("ankle_torque"))
    {
      ankleTorqueSqrt = std::sqrt(static_cast<double>(config("ankle_torque")));
    }
    if(config.has("net_wrench"))
    {
      netWrenchSqrt = std::sqrt(static_cast<double>(config.has("net_wrench")));
    }
    if(config.has("pressure"))
    {
      pressureSqrt = std::sqrt(static_cast<double>(config("pressure")));
    }
  }

  mc_rtc::Configuration save() const
  {
    mc_rtc::Configuration config;
    config.add("ankle_torque", std::pow(ankleTorqueSqrt, 2));
    config.add("net_wrench", std::pow(netWrenchSqrt, 2));
    config.add("pressure", std::pow(pressureSqrt, 2));
    return config;
  }
};

/**
 * @brief Stabilizer safety thresholds
 *
 * The corresponding stabilization entries should be clamped within those limits
 *
 * \warning Developper note: Do not change the default thresholds here, it is likely
 * that robot modules and users do not override every single parameter value,
 * and modifying their default might have serious consequences.
 */
struct SafetyThresholds
{
  double MAX_AVERAGE_DCM_ERROR = 0.05; /**< Maximum average (integral) DCM error in [m] */
  double MAX_COP_ADMITTANCE = 0.1; /**< Maximum CoP admittance for foot damping control */
  double MAX_DCM_D_GAIN = 2.; /**< Maximum DCM derivative gain (no unit) */
  double MAX_DCM_I_GAIN = 100.; /**< Maximum DCM average integral gain in [Hz] */
  double MAX_DCM_P_GAIN = 20.; /**< Maximum DCM proportional gain in [Hz] */
  double MAX_DFZ_ADMITTANCE = 5e-4; /**< Maximum admittance in [s] / [kg] for foot force difference control */
  double MAX_DFZ_DAMPING = 10.; /**< Maximum normalized damping in [Hz] for foot force difference control */
  double MAX_FDC_RX_VEL = 0.2; /**< Maximum x-axis angular velocity in [rad] / [s] for foot damping control. */
  double MAX_FDC_RY_VEL = 0.2; /**< Maximum y-axis angular velocity in [rad] / [s] for foot damping control. */
  double MAX_FDC_RZ_VEL = 0.2; /**< Maximum z-axis angular velocity in [rad] / [s] for foot damping control. */
  double MIN_DS_PRESSURE = 15.; /**< Minimum normal contact force in DSP, used to avoid low-pressure
                                                    targets when close to contact switches. */
  /**< Minimum force for valid ZMP computation (throws otherwise) */
  double MIN_NET_TOTAL_FORCE_ZMP = 1.;

  void load(const mc_rtc::Configuration & config)
  {
    config("MAX_AVERAGE_DCM_ERROR", MAX_AVERAGE_DCM_ERROR);
    config("MAX_COP_ADMITTANCE", MAX_COP_ADMITTANCE);
    config("MAX_DCM_D_GAIN", MAX_DCM_D_GAIN);
    config("MAX_DCM_I_GAIN", MAX_DCM_I_GAIN);
    config("MAX_DCM_P_GAIN", MAX_DCM_P_GAIN);
    config("MAX_DFZ_ADMITTANCE", MAX_DFZ_ADMITTANCE);
    config("MAX_DFZ_DAMPING", MAX_DFZ_DAMPING);
    config("MAX_FDC_RX_VEL", MAX_FDC_RX_VEL);
    config("MAX_FDC_RY_VEL", MAX_FDC_RY_VEL);
    config("MAX_FDC_RZ_VEL", MAX_FDC_RZ_VEL);
    config("MIN_DS_PRESSURE", MIN_DS_PRESSURE);
    config("MIN_NET_TOTAL_FORCE_ZMP", MIN_NET_TOTAL_FORCE_ZMP);
  }

  mc_rtc::Configuration save() const
  {
    mc_rtc::Configuration config;
    config.add("MAX_AVERAGE_DCM_ERROR", MAX_AVERAGE_DCM_ERROR);
    config.add("MAX_COP_ADMITTANCE", MAX_COP_ADMITTANCE);
    config.add("MAX_DCM_D_GAIN", MAX_DCM_D_GAIN);
    config.add("MAX_DCM_I_GAIN", MAX_DCM_I_GAIN);
    config.add("MAX_DCM_P_GAIN", MAX_DCM_P_GAIN);
    config.add("MAX_DFZ_ADMITTANCE", MAX_DFZ_ADMITTANCE);
    config.add("MAX_DFZ_DAMPING", MAX_DFZ_DAMPING);
    config.add("MAX_FDC_RX_VEL", MAX_FDC_RX_VEL);
    config.add("MAX_FDC_RY_VEL", MAX_FDC_RY_VEL);
    config.add("MAX_FDC_RZ_VEL", MAX_FDC_RZ_VEL);
    config.add("MIN_DS_PRESSURE", MIN_DS_PRESSURE);
    config.add("MIN_NET_TOTAL_FORCE_ZMP", MIN_NET_TOTAL_FORCE_ZMP);
    return config;
  }
};

/** Parameters for the DCM bias estimator */
struct DCMBiasEstimatorConfiguration
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /// the standard deviation of the dcm estimation error, NOT including the bias [m]
  double dcmMeasureErrorStd = 0.01;
  /// the standard deviaiton of the zmp estimation error [m]
  double zmpMeasureErrorStd = 0.05;
  /// the standard deviation of the drift [m/s]
  double biasDriftPerSecondStd = 0.02;
  /// Maximum bias in the sagital and lateral directions [m]
  Eigen::Vector2d biasLimit = {0.02, 0.02};
  /// Whether the DCM bias estimator is enabled (default: false for backwards compatibility)
  bool withDCMBias = false;
  /// Whether the DCM filter is enabled
  bool withDCMFilter = false;

  void load(const mc_rtc::Configuration & config)
  {
    config("dcmMeasureErrorStd", dcmMeasureErrorStd);
    config("zmpMeasureErrorStd", zmpMeasureErrorStd);
    config("biasDriftPerSecondStd", biasDriftPerSecondStd);
    config("biasLimit", biasLimit);
    config("withDCMBias", withDCMBias);
    config("withDCMFilter", withDCMFilter);
  }

  mc_rtc::Configuration save() const
  {
    mc_rtc::Configuration config;
    config.add("dcmMeasureErrorStd", dcmMeasureErrorStd);
    config.add("zmpMeasureErrorStd", zmpMeasureErrorStd);
    config.add("biasDriftPerSecondStd", biasDriftPerSecondStd);
    config.add("biasLimit", biasLimit);
    config.add("withDCMBias", withDCMBias);
    config.add("withDCMFilter", withDCMFilter);
    return config;
  }
};

} // namespace lipm_stabilizer
} // namespace mc_rbdyn

namespace mc_rtc
{
/**
 * @brief Read force distribution QP weights from configuration.
 */
template<>
struct ConfigurationLoader<mc_rbdyn::lipm_stabilizer::FDQPWeights>
{
  static mc_rbdyn::lipm_stabilizer::FDQPWeights load(const mc_rtc::Configuration & config)
  {
    mc_rbdyn::lipm_stabilizer::FDQPWeights weights;
    weights.load(config);
    return weights;
  }

  static mc_rtc::Configuration save(const mc_rbdyn::lipm_stabilizer::FDQPWeights & weights)
  {
    return weights.save();
  }
};

/**
 * @brief Read-write stabilizer safety thresholds from configuration
 */
template<>
struct ConfigurationLoader<mc_rbdyn::lipm_stabilizer::SafetyThresholds>
{
  static mc_rbdyn::lipm_stabilizer::SafetyThresholds load(const mc_rtc::Configuration & config)
  {
    mc_rbdyn::lipm_stabilizer::SafetyThresholds safety;
    safety.load(config);
    return safety;
  }

  static mc_rtc::Configuration save(const mc_rbdyn::lipm_stabilizer::SafetyThresholds & safety)
  {
    return safety.save();
  }
};

/**
 * @brief Read DCMBias estimation parameters
 */
template<>
struct ConfigurationLoader<mc_rbdyn::lipm_stabilizer::DCMBiasEstimatorConfiguration>
{
  static mc_rbdyn::lipm_stabilizer::DCMBiasEstimatorConfiguration load(const mc_rtc::Configuration & config)
  {
    mc_rbdyn::lipm_stabilizer::DCMBiasEstimatorConfiguration bias;
    bias.load(config);
    return bias;
  }

  static mc_rtc::Configuration save(const mc_rbdyn::lipm_stabilizer::DCMBiasEstimatorConfiguration & bias)
  {
    return bias.save();
  }
};
} // namespace mc_rtc

namespace mc_rbdyn
{
namespace lipm_stabilizer
{

/**
 * @brief Configuration of the LIPMStabilizer. This configuration is meant to be
 * overriden from the RobotModule, and the user YAML configuration of the
 * stabilizer task.
 *
 * \note Developper note: Do not change the default gains here, it is likely
 * that robot modules and users do not override every single parameter value,
 * and modifying their default might have serious consequences.
 */
struct MC_RBDYN_DLLAPI StabilizerConfiguration
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SafetyThresholds safetyThresholds;
  FDQPWeights fdqpWeights;

  double friction = 0.7; /**< Friction coefficient. Same for both feet */
  std::string leftFootSurface; /**< Surface name for the left foot. Origin should be at foot's center */
  std::string rightFootSurface; /**< Surface name for the right foot. Origin should be at foot's center */

  Eigen::Vector2d copAdmittance = Eigen::Vector2d::Zero(); /**< Admittance gains for foot damping control */
  sva::MotionVecd copMaxVel{{0.3, 0.3, 0.3}, {0.1, 0.1, 0.1}}; /**< Maximal velocity of the cop tasks */
  double copVelFilterGain = 0.8; /**< Gain of the low-pass filter on the cop task reference velocity */
  ZMPCCConfiguration zmpcc; /**< Configuration of ZMPCC (CoM admittance) */

  double dfzAdmittance = 1e-4; /**< Admittance for foot force difference control */
  double dfzDamping = 0.; /**< Damping term in foot force difference control */

  double dcmPropGain = 1.; /**< Proportional gain on DCM error */
  double dcmIntegralGain = 5.; /**< Integral gain on DCM error */
  double dcmDerivGain = 0.; /**< Derivative gain on DCM error */
  double dcmIntegratorTimeConstant =
      15.; /**< Time window for exponential moving average filter of the DCM integrator */
  double dcmDerivatorTimeConstant = 1.; /**< Time window for the stationary offset filter of the DCM derivator */

  std::vector<std::string> comActiveJoints; /**< Joints used by CoM IK task */
  Eigen::Vector3d comStiffness = {1000., 1000., 100.}; /**< Stiffness of CoM IK task */
  double comWeight = 1000.; /**< Weight of CoM IK task */
  double comHeight = 0.84; /**< Desired height of the CoM */

  std::string torsoBodyName; /**< Name of the torso body */
  double torsoPitch = 0; /**< Target world pitch angle for the torso */
  double torsoStiffness = 10; /**< Stiffness of the torso task. */
  double torsoWeight = 100; /**< Weight of the torso task. Should be much lower than CoM and Contacts */
  double pelvisStiffness = 10; /**< Stiffness of the pelvis task. */
  double pelvisWeight = 100; /**< Weight of the torso task. Should be much lower than CoM and Contacts */

  sva::MotionVecd contactDamping{{300, 300, 300},
                                 {300, 300, 300}}; /**< Damping coefficients of the contacts CoP tasks */
  sva::MotionVecd contactStiffness = {{1, 1, 1}, {1, 1, 1}}; /**< Stiffness coefficients of the contacts CoP tasks */
  double contactWeight = 100000.; /**< Weight of contact IK tasks */

  double vdcFrequency = 1.; /**< Frequency used in double-support vertical drift compensation */
  double vdcStiffness = 1000.; /**< Stiffness used in single-support vertical drift compensation */

  DCMBiasEstimatorConfiguration dcmBias; /**< Parameters for the DCM bias estimation */

  StabilizerConfiguration() {}

  StabilizerConfiguration(const mc_rtc::Configuration & conf)
  {
    load(conf);
  }

  /**
   * @brief Checks that the chosen parameters are within the parameters defined
   * by the SafetyThresholds
   */
  void clampGains()
  {
    using ::mc_filter::utils::clampInPlaceAndWarn;
    const auto & s = safetyThresholds;
    clampInPlaceAndWarn(copAdmittance.x(), 0., s.MAX_COP_ADMITTANCE, "CoP x-admittance");
    clampInPlaceAndWarn(copAdmittance.y(), 0., s.MAX_COP_ADMITTANCE, "CoP y-admittance");
    clampInPlaceAndWarn(dcmDerivGain, 0., s.MAX_DCM_D_GAIN, "DCM deriv x-gain");
    clampInPlaceAndWarn(dcmIntegralGain, 0., s.MAX_DCM_I_GAIN, "DCM integral x-gain");
    clampInPlaceAndWarn(dcmPropGain, 0., s.MAX_DCM_P_GAIN, "DCM prop x-gain");
    clampInPlaceAndWarn(dfzAdmittance, 0., s.MAX_DFZ_ADMITTANCE, "DFz admittance");
    clampInPlaceAndWarn(dfzDamping, 0., s.MAX_DFZ_DAMPING, "DFz admittance");
  }

  void load(const mc_rtc::Configuration & config)
  {
    if(config.has("safety_tresholds"))
    {
      safetyThresholds.load(config("safety_tresholds"));
    }

    if(config.has("fdqp_weights"))
    {
      fdqpWeights.load(config("fdqp_weights"));
    }

    config("leftFootSurface", leftFootSurface);
    config("rightFootSurface", rightFootSurface);
    config("torsoBodyName", torsoBodyName);
    config("friction", friction);

    if(config.has("admittance"))
    {
      auto admittance = config("admittance");
      admittance("cop", copAdmittance);
      admittance("maxVel", copMaxVel);
      admittance("velFilterGain", mc_filter::utils::clamp(copVelFilterGain, 0, 1));
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
    if(config.has("dcm_bias"))
    {
      dcmBias.load(config("dcm_bias"));
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
      }

      if(tasks.has("torso"))
      {

        tasks("torso")("pitch", torsoPitch);
        tasks("torso")("stiffness", torsoStiffness);
        tasks("torso")("weight", torsoWeight);
      }

      if(tasks.has("pelvis"))
      {
        tasks("pelvis")("stiffness", pelvisStiffness);
        tasks("pelvis")("weight", pelvisWeight);
      }

      if(tasks.has("contact"))
      {
        if(tasks("contact").has("damping"))
        {
          try
          {
            double d = tasks("contact")("damping");
            contactDamping = sva::MotionVecd({d, d, d}, {d, d, d});
          }
          catch(mc_rtc::Configuration::Exception & e)
          {
            e.silence();
            contactDamping = tasks("contact")("damping");
          }
        }
        if(tasks("contact").has("stiffness"))
        {
          try
          {
            double k = tasks("contact")("stiffness");
            contactStiffness = sva::MotionVecd({k, k, k}, {k, k, k});
          }
          catch(mc_rtc::Configuration::Exception & e)
          {
            e.silence();
            contactStiffness = tasks("contact")("stiffness");
          }
        }
        tasks("contact")("stiffness", contactStiffness);
        tasks("contact")("weight", contactWeight);
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
      zmpcc.load(config("zmpcc"));
    }
    config("zmpcc", zmpcc);
  }

  mc_rtc::Configuration save() const
  {
    mc_rtc::Configuration conf;
    conf.add("safety_tresholds", safetyThresholds);
    conf.add("fdqp_weights", fdqpWeights);

    conf.add("torsoBodyName", torsoBodyName);
    conf.add("leftFootSurface", leftFootSurface);
    conf.add("rightFootSurface", rightFootSurface);

    conf.add("admittance");
    conf("admittance").add("cop", copAdmittance);
    conf("admittance").add("dfz", dfzAdmittance);
    conf("admittance").add("dfz_damping", dfzDamping);
    conf("admittance").add("maxVel", copMaxVel);
    conf("admittance").add("velFilterGain", copVelFilterGain);

    conf.add("zmpcc", zmpcc);

    conf.add("dcm_tracking");
    conf("dcm_tracking").add("gains");
    conf("dcm_tracking")("gains").add("prop", dcmPropGain);
    conf("dcm_tracking")("gains").add("integral", dcmIntegralGain);
    conf("dcm_tracking")("gains").add("deriv", dcmDerivGain);
    conf("dcm_tracking").add("derivator_time_constant", dcmDerivatorTimeConstant);
    conf("dcm_tracking").add("integrator_time_constant", dcmIntegratorTimeConstant);

    conf.add("dcm_bias", dcmBias);

    conf.add("tasks");
    conf("tasks").add("com");
    conf("tasks")("com").add("active_joints", comActiveJoints);
    conf("tasks")("com").add("stiffness", comStiffness);
    conf("tasks")("com").add("weight", comWeight);
    conf("tasks")("com").add("height", comHeight);

    conf("tasks").add("torso");
    conf("tasks")("torso").add("pitch", torsoPitch);
    conf("tasks")("torso").add("stiffness", torsoStiffness);
    conf("tasks")("torso").add("weight", torsoWeight);

    conf("tasks").add("pelvis");
    conf("tasks")("pelvis").add("stiffness", pelvisStiffness);
    conf("tasks")("pelvis").add("weight", pelvisWeight);

    conf("tasks").add("contact");
    conf("tasks")("contact").add("damping", contactDamping);
    conf("tasks")("contact").add("stiffness", contactStiffness);
    conf("tasks")("contact").add("weight", contactWeight);

    conf.add("vdc");
    conf("vdc").add("frequency", vdcFrequency);
    conf("vdc").add("stiffness", vdcStiffness);
    return conf;
  }
};
} // namespace lipm_stabilizer
} // namespace mc_rbdyn

namespace mc_rtc
{
template<>
struct ConfigurationLoader<mc_rbdyn::lipm_stabilizer::StabilizerConfiguration>
{
  static mc_rbdyn::lipm_stabilizer::StabilizerConfiguration load(const mc_rtc::Configuration & config)
  {
    mc_rbdyn::lipm_stabilizer::StabilizerConfiguration stabi;
    stabi.load(config);
    return stabi;
  }

  static mc_rtc::Configuration save(const mc_rbdyn::lipm_stabilizer::StabilizerConfiguration & stabiConf)
  {
    return stabiConf.save();
  }
};
} // namespace mc_rtc
