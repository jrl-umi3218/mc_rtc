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
  FDQPWeights(double netWrench, double ankleTorque, double pressure)
  : ankleTorqueSqrt(std::sqrt(ankleTorque)), netWrenchSqrt(std::sqrt(netWrench)), pressureSqrt(std::sqrt(pressure))
  {
  }
  double ankleTorqueSqrt;
  double netWrenchSqrt;
  double pressureSqrt;
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
    double ankleTorqueWeight = config("ankle_torque");
    double netWrenchWeight = config("net_wrench");
    double pressureWeight = config("pressure");
    weights.ankleTorqueSqrt = std::sqrt(ankleTorqueWeight);
    weights.netWrenchSqrt = std::sqrt(netWrenchWeight);
    weights.pressureSqrt = std::sqrt(pressureWeight);
    return weights;
  }

  static mc_rtc::Configuration save(const mc_rbdyn::lipm_stabilizer::FDQPWeights & weights)
  {
    mc_rtc::Configuration config;
    config.add("ankle_torque", std::pow(weights.ankleTorqueSqrt, 2));
    config.add("net_wrench", std::pow(weights.netWrenchSqrt, 2));
    config.add("pressure", std::pow(weights.pressureSqrt, 2));
    return config;
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
    config("MAX_AVERAGE_DCM_ERROR", safety.MAX_AVERAGE_DCM_ERROR);
    config("MAX_COP_ADMITTANCE", safety.MAX_COP_ADMITTANCE);
    config("MAX_DCM_D_GAIN", safety.MAX_DCM_D_GAIN);
    config("MAX_DCM_I_GAIN", safety.MAX_DCM_I_GAIN);
    config("MAX_DCM_P_GAIN", safety.MAX_DCM_P_GAIN);
    config("MAX_DFZ_ADMITTANCE", safety.MAX_DFZ_ADMITTANCE);
    config("MAX_DFZ_DAMPING", safety.MAX_DFZ_DAMPING);
    config("MAX_FDC_RX_VEL", safety.MAX_FDC_RX_VEL);
    config("MAX_FDC_RY_VEL", safety.MAX_FDC_RY_VEL);
    config("MAX_FDC_RZ_VEL", safety.MAX_FDC_RZ_VEL);
    config("MIN_DS_PRESSURE", safety.MIN_DS_PRESSURE);
    config("MIN_NET_TOTAL_FORCE_ZMP", safety.MIN_NET_TOTAL_FORCE_ZMP);
    return safety;
  }

  static mc_rtc::Configuration save(const mc_rbdyn::lipm_stabilizer::SafetyThresholds & safety)
  {
    mc_rtc::Configuration config;
    config.add("MAX_AVERAGE_DCM_ERROR", safety.MAX_AVERAGE_DCM_ERROR);
    config.add("MAX_COP_ADMITTANCE", safety.MAX_COP_ADMITTANCE);
    config.add("MAX_DCM_D_GAIN", safety.MAX_DCM_D_GAIN);
    config.add("MAX_DCM_I_GAIN", safety.MAX_DCM_I_GAIN);
    config.add("MAX_DCM_P_GAIN", safety.MAX_DCM_P_GAIN);
    config.add("MAX_DFZ_ADMITTANCE", safety.MAX_DFZ_ADMITTANCE);
    config.add("MAX_DFZ_DAMPING", safety.MAX_DFZ_DAMPING);
    config.add("MAX_FDC_RX_VEL", safety.MAX_FDC_RX_VEL);
    config.add("MAX_FDC_RY_VEL", safety.MAX_FDC_RY_VEL);
    config.add("MAX_FDC_RZ_VEL", safety.MAX_FDC_RZ_VEL);
    config.add("MIN_DS_PRESSURE", safety.MIN_DS_PRESSURE);
    config.add("MIN_NET_TOTAL_FORCE_ZMP", safety.MIN_NET_TOTAL_FORCE_ZMP);
    return config;
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
    config("dcmMeasureErrorStd", bias.dcmMeasureErrorStd);
    config("zmpMeasureErrorStd", bias.zmpMeasureErrorStd);
    config("biasDriftPerSeconStd", bias.biasDriftPerSecondStd);
    config("biasLimit", bias.biasLimit);
    return bias;
  }

  static mc_rtc::Configuration save(const mc_rbdyn::lipm_stabilizer::DCMBiasEstimatorConfiguration & bias)
  {
    mc_rtc::Configuration config;
    config.add("dcmMeasureErrorStd", bias.dcmMeasureErrorStd);
    config.add("zmpMeasureErrorStd", bias.zmpMeasureErrorStd);
    config.add("biasDriftPerSeconStd", bias.biasDriftPerSecondStd);
    config.add("biasLimit", bias.biasLimit);
    return config;
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
    config("safety_tresholds", safetyThresholds);

    config("fdqp_weights", fdqpWeights);
    config("leftFootSurface", leftFootSurface);
    config("rightFootSurface", rightFootSurface);
    config("torsoBodyName", torsoBodyName);
    config("friction", friction);

    if(config.has("admittance"))
    {
      auto admittance = config("admittance");
      admittance("cop", copAdmittance);
      admittance("maxVel", copMaxVel);
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
          double d = tasks("contact")("damping");
          contactDamping = sva::MotionVecd({d, d, d}, {d, d, d});
        }
        if(tasks("contact").has("stiffness"))
        {
          double k = tasks("contact")("stiffness");
          contactStiffness = sva::MotionVecd({k, k, k}, {k, k, k});
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

    config("zmpcc", zmpcc);
    config("dcmBias", dcmBias);
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

    conf.add("zmpcc", zmpcc);

    conf.add("dcm_tracking");
    conf("dcm_tracking").add("gains");
    conf("dcm_tracking")("gains").add("prop", dcmPropGain);
    conf("dcm_tracking")("gains").add("integral", dcmIntegralGain);
    conf("dcm_tracking")("gains").add("deriv", dcmDerivGain);
    conf("dcm_tracking").add("derivator_time_constant", dcmDerivatorTimeConstant);
    conf("dcm_tracking").add("integrator_time_constant", dcmIntegratorTimeConstant);

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

    conf.add("dcmBias", dcmBias);
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
