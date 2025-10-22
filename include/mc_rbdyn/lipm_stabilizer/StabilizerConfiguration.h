/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Gains.h>
#include <mc_rbdyn/api.h>
#include <mc_rbdyn/lipm_stabilizer/ZMPCCConfiguration.h>

#include <mc_filter/utils/clamp.h>

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
    if(config.has("ankle_torque")) { ankleTorqueSqrt = std::sqrt(static_cast<double>(config("ankle_torque"))); }
    if(config.has("net_wrench")) { netWrenchSqrt = std::sqrt(static_cast<double>(config("net_wrench"))); }
    if(config.has("pressure")) { pressureSqrt = std::sqrt(static_cast<double>(config("pressure"))); }
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

/** Weights for CoP distribution over horizon (FDMPC). */
struct MC_RBDYN_DLLAPI FDMPCWeights
{

  FDMPCWeights()
  {
    cop_ = 100;
    copRegulation_ = 1e-8;
    copDiff_ = 1e-8;
  }

  FDMPCWeights(double cop, double copReg, double copDiff)
  {
    cop_ = cop;
    copRegulation_ = copReg;
    copDiff_ = copDiff;
  }
  double cop_;
  double copRegulation_;
  double copDiff_;

  void load(const mc_rtc::Configuration & config)
  {
    if(config.has("cop")) { cop_ = static_cast<double>(config("cop")); }
    if(config.has("cop_regulation")) { copRegulation_ = static_cast<double>(config("cop_regulation")); }
    if(config.has("cop_diff")) { copDiff_ = static_cast<double>(config("cop_diff")); }
  }

  mc_rtc::Configuration save() const
  {
    mc_rtc::Configuration config;
    config.add("cop", cop_);
    config.add("cop_regulation", copRegulation_);
    config.add("cop_diff", copDiff_);
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
  double MAX_COMD_GAIN = 10.; /**< Maximum CoMd gain in [Hz] */
  double MAX_ZMPD_GAIN = 10.; /**< Maximum ZMPd gain in [Hz] */
  double MAX_DF_ADMITTANCE = 5e-4; /**< Maximum admittance in [s] / [kg] for foot force difference control */
  double MAX_DF_DAMPING = 10.; /**< Maximum normalized damping in [Hz] for foot force difference control */
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
    config("MAX_COMD_GAIN", MAX_COMD_GAIN);
    config("MAX_ZMPD_GAIN", MAX_ZMPD_GAIN);
    config("MAX_DF_ADMITTANCE", MAX_DF_ADMITTANCE);
    if(config.has("MAX_DFZ_ADMITTANCE"))
    {
      mc_rtc::log::deprecated("StabilizerConfiguration::SafetyThresholds", "MAX_DFZ_ADMITTANCE", "MAX_DF_ADMITTANCE");
      config("MAX_DFZ_ADMITTANCE", MAX_DF_ADMITTANCE);
    }
    config("MAX_DF_DAMPING", MAX_DF_ADMITTANCE);
    if(config.has("MAX_DFZ_DAMPING"))
    {
      mc_rtc::log::deprecated("StabilizerConfiguration::SafetyThresholds", "MAX_DFZ_DAMPING", "MAX_DF_ADMITTANCE");
      config("MAX_DFZ_DAMPING", MAX_DF_ADMITTANCE);
    }
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
    config.add("MAX_COMD_GAIN", MAX_COMD_GAIN);
    config.add("MAX_ZMPD_GAIN", MAX_ZMPD_GAIN);
    config.add("MAX_DF_ADMITTANCE", MAX_DF_ADMITTANCE);
    config.add("MAX_DF_DAMPING", MAX_DF_DAMPING);
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
  /// Maximum bias in the sagital and lateral directions used to correct the CoM, should be smaller than bisaLimit [m]
  Eigen::Vector2d comBiasLimit = biasLimit;
  /// Whether the DCM bias estimator is enabled (default: false for backwards compatibility)
  bool withDCMBias = false;
  /// Whether the DCM filter is enabled
  bool withDCMFilter = false;
  /// Whether the absolute CoM position gets unbiased (useful when the disturbance comes from a bias on the CoM, for
  /// example modeling errors or carrying an object, but it would be wrong if the disturbance comes from an unexpected
  /// contact, the estimator cannot distinguish between them)
  bool correctCoMPos = false;
  /// Whether the estimated bias should be used on the dcm
  bool correctDCM = true;

  void load(const mc_rtc::Configuration & config)
  {
    config("dcmMeasureErrorStd", dcmMeasureErrorStd);
    config("zmpMeasureErrorStd", zmpMeasureErrorStd);
    config("biasDriftPerSecondStd", biasDriftPerSecondStd);
    config("biasLimit", biasLimit);
    config("comBiasLimit", comBiasLimit);
    config("withDCMBias", withDCMBias);
    config("correctCoMPos", correctCoMPos);
    config("withDCMFilter", withDCMFilter);
    config("correctDCM", correctDCM);
  }

  mc_rtc::Configuration save() const
  {
    mc_rtc::Configuration config;
    config.add("dcmMeasureErrorStd", dcmMeasureErrorStd);
    config.add("zmpMeasureErrorStd", zmpMeasureErrorStd);
    config.add("biasDriftPerSecondStd", biasDriftPerSecondStd);
    config.add("biasLimit", biasLimit);
    config.add("comBiasLimit", comBiasLimit);
    config.add("withDCMBias", withDCMBias);
    config.add("correctCoMPos", correctCoMPos);
    config.add("correctDCM", correctDCM);
    config.add("withDCMFilter", withDCMFilter);
    return config;
  }
};

/** @brief Parameters for the external wrenches
 *
 *  The external forces affect the two layers, i.e., pattern generation and stabilization control.
 *
 *  For the pattern generation, if the external forces are not taken into account in the input target CoM, an offset is
 * added by feedforward from the target external forces. This feature is enabled by setting addExpectedCoMOffset to
 * true. For the stabilization control, the error between the target external forces and the measured external forces is
 * compensated by the CoM strategy and the ZMP strategy. High-frequency errors are handled by the ZMP strategy, while
 * low-frequency errors are handled by the CoM strategy. The CoM strategy is enabled when modifyCoMErr is set to true.
 * The ZMP strategy is enabled when modifyZMPErr is set to true. Strictly speaking, the ZMP strategy also depends on the
 * derivative of the external force error, and when modifyZMPErrD is true, the corresponding compensation is also
 * enabled. However, the effect of the compensation related to the derivative is not dominant and is affected by the
 * measured external force noise, so it is recommended to try with modifyZMPErrD false first.
 *
 * If there are additional biases and unknown external forces that are not predicted nor measured, the DCM bias
 * estimator could be used. In that case, if one of addExpectedCoMOffset, modifyCoMErr or modifyZMPErr is true, it is
 * highly recommended to set excludeFromDCMBiasEst to true. It will exclude the expected/measured external forces from
 * the computation of the bias and prevent them to be twice compensated for.
 *
 * CommOffsetDerivatorTimeConstant is a time constant for calculating this derivative. The maximum amounts of
 * modification for CoM strategy and ZMP strategy are comOffsetErrCoMLimit and comOffsetErrZMPLimit, respectively.
 *
 *  The total measured external force is passed through the low-pass filter of the cutoff period
 * exitWenchSumLowPassCutoffPeriod. Then, from the error between the target external forces and the measured external
 * forces, the amounts of compensation for the CoM strategy and ZMP strategy are calculated using two low-pass filters.
 * One is a low-pass filter of the cutoff period comOffsetLowPassCutoffPeriod (High-LPF), and the other is a low-pass
 * filter of the cutoff period comOffsetLowPassCoMCutoffPeriod (Low-LPF). Make the comOffsetLowPassCoMCutoffPeriod
 * larger than the comOffsetLowPassCutoffPeriod. The CoM strategy deals with the error components that pass through the
 * High-LPF and pass through the Low-LPF. The ZMP strategy deals with the error components that pass the High-LPF but do
 * NOT pass the Low-LPF.
 *
 *  - If you want the StabilizerTask to deal with external forces in a simple way (i.e., the pattern generator does not
 *    take external forces into account), then set addExpectedCoMOffset, modifyCoMErr, modifyZMPErr to true.
 *  - modifyZMPErrD should theoretically be true, but it depends on the derivative of the measured external forces, so
 *    it becomes sensitive to the noises in the measurement noise.
 *  - If you want to use the DCM bias estimator then if the pattern generator takes the forces into account or if one of
 * addExpectedCoMOffset, modifyCoMErr or modifyZMPErr is true, set excludeFromDCMBiasEst to true.
 *  - subtractMeasuredValue is a more experimental and the option inspired from
 *    https://github.com/stephane-caron/lipm_walking_controller/discussions/28 Normally set to false.
 */
struct ExternalWrenchConfiguration
{
  /// Whether to add the CoM offset expected from the external wrenches.
  /// Should be false if the target CoM generated by the pattern generator already considers the external wrenches, true
  /// otherwise.
  bool addExpectedCoMOffset = false;
  /// Subtract the measured external wrenches instead of target ones
  bool subtractMeasuredValue = false;
  /// Modify CoM depending on the error of the external wrenches in target and measurement
  bool modifyCoMErr = false;
  /// Modify ZMP depending on the error of the external wrenches in target and measurement
  bool modifyZMPErr = false;
  /// Modify ZMP velocity depending on the error velocity of the external wrenches in target and measurement
  bool modifyZMPErrD = false;
  /// Exclude external forces from DCM Bias estimation (should be used when the pg/stabilizer is actively compensating
  /// for those measured forces)
  bool excludeFromDCMBiasEst = false;
  /// Limit of CoM offset error handled by CoM modification
  double comOffsetErrCoMLimit = 0.1;
  /// Limit of CoM offset error handled by ZMP modification [m]
  double comOffsetErrZMPLimit = 0.1;
  /// Cutoff period for the low-pass filter of the sum of the measured external wrenches
  double extWrenchSumLowPassCutoffPeriod = 0.05;
  /// Cutoff period for the low-pass filter of CoM offset
  double comOffsetLowPassCutoffPeriod = 0.05;
  /// Cutoff period for the low-pass filter of CoM offset to extract CoM modification
  double comOffsetLowPassCoMCutoffPeriod = 1.0;
  /// Time window for the stationary offset filter of the CoM offset derivator
  double comOffsetDerivatorTimeConstant = 1.0;

  void load(const mc_rtc::Configuration & config)
  {
    config("add_expected_com_offset", addExpectedCoMOffset);
    config("subtract_measured_value", subtractMeasuredValue);
    config("modify_com_error", modifyCoMErr);
    config("modify_zmp_error", modifyZMPErr);
    config("modify_zmp_error_d", modifyZMPErrD);
    config("exclude_from_dcm_bias_est", excludeFromDCMBiasEst);
    config("com_offset_err_com_limit", comOffsetErrCoMLimit);
    config("com_offset_err_zmp_limit", comOffsetErrZMPLimit);
    config("ext_wrench_sum_cutoff", extWrenchSumLowPassCutoffPeriod);
    config("com_offset_cutoff", comOffsetLowPassCutoffPeriod);
    config("com_offset_com_cutoff", comOffsetLowPassCoMCutoffPeriod);
    config("derivator_time_constant", comOffsetDerivatorTimeConstant);
  }

  mc_rtc::Configuration save() const
  {
    mc_rtc::Configuration config;
    config.add("add_expected_com_offset", addExpectedCoMOffset);
    config.add("subtract_measured_value", subtractMeasuredValue);
    config.add("modify_com_error", modifyCoMErr);
    config.add("modify_zmp_error", modifyZMPErr);
    config.add("modify_zmp_error_d", modifyZMPErrD);
    config.add("exclude_from_dcm_bias_est", excludeFromDCMBiasEst);
    config.add("com_offset_err_com_limit", comOffsetErrCoMLimit);
    config.add("com_offset_err_zmp_limit", comOffsetErrZMPLimit);
    config.add("ext_wrench_sum_cutoff", extWrenchSumLowPassCutoffPeriod);
    config.add("com_offset_cutoff", comOffsetLowPassCutoffPeriod);
    config.add("com_offset_com_cutoff", comOffsetLowPassCoMCutoffPeriod);
    config.add("derivator_time_constant", comOffsetDerivatorTimeConstant);
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

  static mc_rtc::Configuration save(const mc_rbdyn::lipm_stabilizer::FDQPWeights & weights) { return weights.save(); }
};

/**
 * @brief Read CoP distribution over horizon QP weights from configuration.
 */
template<>
struct ConfigurationLoader<mc_rbdyn::lipm_stabilizer::FDMPCWeights>
{
  static mc_rbdyn::lipm_stabilizer::FDMPCWeights load(const mc_rtc::Configuration & config)
  {
    mc_rbdyn::lipm_stabilizer::FDMPCWeights weights;
    weights.load(config);
    return weights;
  }

  static mc_rtc::Configuration save(const mc_rbdyn::lipm_stabilizer::FDMPCWeights & weights) { return weights.save(); }
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

/**
 * @brief Read parameters for the external wrenches
 */
template<>
struct ConfigurationLoader<mc_rbdyn::lipm_stabilizer::ExternalWrenchConfiguration>
{
  static mc_rbdyn::lipm_stabilizer::ExternalWrenchConfiguration load(const mc_rtc::Configuration & config)
  {
    mc_rbdyn::lipm_stabilizer::ExternalWrenchConfiguration extWrench;
    extWrench.load(config);
    return extWrench;
  }

  static mc_rtc::Configuration save(const mc_rbdyn::lipm_stabilizer::ExternalWrenchConfiguration & extWrench)
  {
    return extWrench.save();
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

  bool verbose = true; /**< Enable verbose output messages */

  SafetyThresholds safetyThresholds;
  FDQPWeights fdqpWeights;
  FDMPCWeights fdmpcWeights;

  double friction = 0.7; /**< Friction coefficient. Same for both feet */
  std::string leftFootSurface; /**< Surface name for the left foot. Origin should be at foot's center */
  std::string rightFootSurface; /**< Surface name for the right foot. Origin should be at foot's center */

  bool constrainCoP = true; /**<If true, the wrench computation constraint the CoP of each contact to be inside the foot
                               support rectangle. */

  bool useTargetPressure = true; /**<If true, the CoP tasks compute the target torque using the target vertical force,
                                    otherwise, the torque is computed using the measured one. */

  Eigen::Vector2d copAdmittance = Eigen::Vector2d::Zero(); /**< Admittance gains for foot damping control */
  sva::MotionVecd copMaxVel{{0.3, 0.3, 0.3}, {0.1, 0.1, 0.1}}; /**< Maximal velocity of the cop tasks */
  double copVelFilterGain = 0.8; /**< Gain of the low-pass filter on the cop task reference velocity */
  ZMPCCConfiguration zmpcc; /**< Configuration of ZMPCC (CoM admittance) */

  mc_rbdyn::Gains3d dfAdmittance = {0, 0, 1e-4}; /**< Admittance for foot force difference control */
  mc_rbdyn::Gains3d dfDamping = 0.; /**< Damping term in foot force difference control */

  mc_rbdyn::Gains2d dcmPropGain = 1.; /**< Proportional gain on DCM error */
  mc_rbdyn::Gains2d dcmIntegralGain = 5.; /**< Integral gain on DCM error */
  mc_rbdyn::Gains2d dcmDerivGain = 0.; /**< Derivative gain on DCM error */
  double comdErrorGain = 1.; /**< Gain on CoMd error */
  double zmpdGain = 0.; /**< Gain on ZMPd */
  double dcmIntegratorTimeConstant =
      15.; /**< Time window for exponential moving average filter of the DCM integrator */
  double dcmDerivatorTimeConstant = 1.; /**< Cutoff Period of the DCM derivator filter */

  double fSumFilterTimeConstant = 20.; /**<Cutoff Period of the estimation of the sum of forces*/

  std::vector<std::string> comActiveJoints; /**< Joints used by CoM IK task */
  Eigen::Vector3d comStiffness = {1000., 1000., 100.}; /**< Stiffness of CoM IK task */
  double comWeight = 1000.; /**< Weight of CoM IK task */
  Eigen::Vector3d comDimWeight = Eigen::Vector3d::Ones(); /**< Dimensional weight of CoM IK task */
  double comHeight = 0.84; /**< Desired height of the CoM */

  /**< 1st order gain constant between a reference CoP and vertical force and the real */
  Eigen::Vector3d copFzLambda = 100.0 * Eigen::Vector3d::Ones();
  double delayCoP = 0;

  std::string torsoBodyName; /**< Name of the torso body */
  double torsoPitch = 0; /**< Target world pitch angle for the torso */
  double torsoStiffness = 10; /**< Stiffness of the torso task. */
  double torsoWeight = 100; /**< Weight of the torso task. Should be much lower than CoM and Contacts */
  Eigen::Vector3d torsoDimWeight = Eigen::Vector3d::Ones(); /**< Dimensional weight of the torso task */
  double pelvisStiffness = 10; /**< Stiffness of the pelvis task. */
  double pelvisWeight = 100; /**< Weight of the pelvis task. Should be much lower than CoM and Contacts */
  Eigen::Vector3d pelvisDimWeight = Eigen::Vector3d::Ones(); /**< Dimensional weight of the pelvis task */

  sva::MotionVecd contactDamping{{300, 300, 300},
                                 {300, 300, 300}}; /**< Damping coefficients of the contacts CoP tasks */
  sva::MotionVecd contactStiffness = {{1, 1, 1}, {1, 1, 1}}; /**< Stiffness coefficients of the contacts CoP tasks */
  double contactWeight = 100000.; /**< Weight of contact IK tasks */

  double vdcFrequency = 1.; /**< Frequency used in double-support vertical drift compensation */
  double vdcStiffness = 1000.; /**< Stiffness used in single-support vertical drift compensation */

  DCMBiasEstimatorConfiguration dcmBias; /**< Parameters for the DCM bias estimation */

  ExternalWrenchConfiguration extWrench; /**< Parameters for the external wrenches */

  StabilizerConfiguration() {}

  StabilizerConfiguration(const mc_rtc::Configuration & conf) { load(conf); }

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
    clampInPlaceAndWarn(dcmDerivGain, 0., s.MAX_DCM_D_GAIN, "DCM deriv gain");
    clampInPlaceAndWarn(dcmIntegralGain, 0., s.MAX_DCM_I_GAIN, "DCM integral gain");
    clampInPlaceAndWarn(dcmPropGain, 0., s.MAX_DCM_P_GAIN, "DCM prop gain");
    clampInPlaceAndWarn(comdErrorGain, 0., s.MAX_COMD_GAIN, "CoMd gain");
    clampInPlaceAndWarn(zmpdGain, 0., s.MAX_ZMPD_GAIN, "ZMPd gain");
    clampInPlaceAndWarn(dfAdmittance, 0., s.MAX_DF_ADMITTANCE, "DF admittance");
    clampInPlaceAndWarn(dfDamping, 0., s.MAX_DF_DAMPING, "DF admittance");
  }

  void load(const mc_rtc::Configuration & config)
  {
    config("verbose", verbose);

    if(config.has("safety_tresholds")) { safetyThresholds.load(config("safety_tresholds")); }

    if(config.has("fdqp_weights")) { fdqpWeights.load(config("fdqp_weights")); }

    if(config.has("fdmpc_weights")) { fdmpcWeights.load(config("fdmpc_weights")); }

    config("friction", friction);
    config("leftFootSurface", leftFootSurface);
    config("rightFootSurface", rightFootSurface);
    config("torsoBodyName", torsoBodyName);

    if(config.has("admittance"))
    {
      auto admittance = config("admittance");
      admittance("cop", copAdmittance);
      admittance("useTargetPressure", useTargetPressure);
      admittance("copFzLambda", copFzLambda);
      admittance("copFzDelay", delayCoP);
      admittance("maxVel", copMaxVel);
      admittance("velFilterGain", mc_filter::utils::clamp(copVelFilterGain, 0, 1));
      if(admittance.has("dfz"))
      {
        mc_rtc::log::warning("[MC_RTC_DEPRECATED][StabilizerConfiguration] dfz is now dimensional, to "
                             "keep the same behavior use df: [0, 0, dfz]");
        dfAdmittance.setZero();
        dfAdmittance.z() = admittance("dfz");
      }
      else
      {
        admittance("df", dfAdmittance);
      }
      if(admittance.has("dfz_damping"))
      {
        mc_rtc::log::warning("[MC_RTC_DEPRECATED][StabilizerConfiguration] dfz_damping is now dimensional, to "
                             "keep the same behavior use df_damping: [0, 0, dfz_damping]");
        dfDamping.setZero();
        dfDamping.z() = admittance("dfz_damping");
      }
      else
      {
        admittance("df_damping", dfDamping);
      }
    }
    if(config.has("dcm_tracking"))
    {
      auto dcmTracking = config("dcm_tracking");
      if(dcmTracking.has("gains"))
      {
        dcmTracking("gains")("prop", dcmPropGain);
        dcmTracking("gains")("integral", dcmIntegralGain);
        dcmTracking("gains")("deriv", dcmDerivGain);
        dcmTracking("gains")("comdError", comdErrorGain);
        dcmTracking("gains")("zmpd", zmpdGain);
      }
      if(dcmTracking.has("derivator_time_constant"))
      {
        mc_rtc::log::warning("derivator_time_constant is deprecated, use derivator_cutoff_period instead");
        dcmTracking("derivator_time_constant", dcmDerivatorTimeConstant);
      }
      dcmTracking("derivator_cutoff_period", dcmDerivatorTimeConstant);
      dcmTracking("integrator_time_constant", dcmIntegratorTimeConstant);
    }
    if(config.has("dcm_bias")) { dcmBias.load(config("dcm_bias")); }
    if(config.has("external_wrench")) { extWrench.load(config("external_wrench")); }
    if(config.has("tasks"))
    {
      auto tasks = config("tasks");
      if(tasks.has("com"))
      {
        tasks("com")("active_joints", comActiveJoints);
        tasks("com")("stiffness", comStiffness);
        tasks("com")("weight", comWeight);
        tasks("com")("dimWeight", comDimWeight);
        tasks("com")("height", comHeight);
      }

      if(tasks.has("torso"))
      {

        tasks("torso")("pitch", torsoPitch);
        tasks("torso")("stiffness", torsoStiffness);
        tasks("torso")("weight", torsoWeight);
        tasks("torso")("dimWeight", torsoDimWeight);
      }

      if(tasks.has("pelvis"))
      {
        tasks("pelvis")("stiffness", pelvisStiffness);
        tasks("pelvis")("weight", pelvisWeight);
        tasks("pelvis")("dimWeight", pelvisDimWeight);
      }

      if(tasks.has("contact"))
      {
        tasks("contact")("constrainCoP", constrainCoP);
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

    if(config.has("zmpcc")) { zmpcc.load(config("zmpcc")); }
    config("zmpcc", zmpcc);
  }

  mc_rtc::Configuration save() const
  {
    mc_rtc::Configuration conf;
    conf.add("verbose", verbose);

    conf.add("safety_tresholds", safetyThresholds);
    conf.add("fdqp_weights", fdqpWeights);
    conf.add("fdmpc_weights", fdmpcWeights);

    conf.add("friction", friction);
    conf.add("leftFootSurface", leftFootSurface);
    conf.add("rightFootSurface", rightFootSurface);
    conf.add("torsoBodyName", torsoBodyName);

    conf.add("admittance");
    conf("admittance").add("cop", copAdmittance);
    conf("admittance").add("useTargetPressure", useTargetPressure);
    conf("admittance").add("df", dfAdmittance);
    conf("admittance").add("df_damping", dfDamping);
    conf("admittance").add("maxVel", copMaxVel);
    conf("admittance").add("velFilterGain", copVelFilterGain);

    conf.add("zmpcc", zmpcc);

    conf.add("dcm_tracking");
    conf("dcm_tracking").add("gains");
    conf("dcm_tracking")("gains").add("prop", dcmPropGain);
    conf("dcm_tracking")("gains").add("integral", dcmIntegralGain);
    conf("dcm_tracking")("gains").add("deriv", dcmDerivGain);
    conf("dcm_tracking")("gains").add("comdError", comdErrorGain);
    conf("dcm_tracking")("gains").add("zmpd", zmpdGain);
    conf("dcm_tracking").add("derivator_cutoff_period", dcmDerivatorTimeConstant);
    conf("dcm_tracking").add("integrator_time_constant", dcmIntegratorTimeConstant);

    conf.add("dcm_bias", dcmBias);
    conf.add("external_wrench", extWrench);

    conf.add("tasks");
    conf("tasks").add("com");
    conf("tasks")("com").add("active_joints", comActiveJoints);
    conf("tasks")("com").add("stiffness", comStiffness);
    conf("tasks")("com").add("weight", comWeight);
    conf("tasks")("com").add("dimWeight", comDimWeight);
    conf("tasks")("com").add("height", comHeight);

    conf("tasks").add("torso");
    conf("tasks")("torso").add("pitch", torsoPitch);
    conf("tasks")("torso").add("stiffness", torsoStiffness);
    conf("tasks")("torso").add("weight", torsoWeight);
    conf("tasks")("torso").add("dimWeight", torsoDimWeight);

    conf("tasks").add("pelvis");
    conf("tasks")("pelvis").add("stiffness", pelvisStiffness);
    conf("tasks")("pelvis").add("weight", pelvisWeight);
    conf("tasks")("pelvis").add("dimWeight", pelvisDimWeight);

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
