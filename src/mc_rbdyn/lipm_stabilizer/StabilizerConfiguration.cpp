#include <mc_rbdyn/lipm_stabilizer/StabilizerConfiguration.h>

#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_rtc/gui/NumberInput.h>

namespace mc_rbdyn
{
namespace lipm_stabilizer
{

void StabilizerConfiguration::clampGains()
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

void StabilizerConfiguration::load(const mc_rtc::Configuration & config)
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

mc_rtc::Configuration StabilizerConfiguration::save() const
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

} // namespace lipm_stabilizer
} // namespace mc_rbdyn
