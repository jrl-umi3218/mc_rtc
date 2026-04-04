#include <mc_rbdyn/gui/lipm_stabilizer/StabilizerConfiguration.h>

#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_rtc/gui/NumberInput.h>

namespace mc_rbdyn::gui::lipm_stabilizer
{

void addStabilizerToGUI(mc_rtc::gui::StateBuilder & gui,
                        const std::vector<std::string> & category,
                        mc_rbdyn::lipm_stabilizer::StabilizerConfiguration & config)
{
  auto category_ = category;
  category_.push_back("Main");
  gui.addElement(
      category_,
      mc_rtc::gui::NumberInput(
          "CoM weight", [&config]() { return config.comWeight; }, [&config](double d) { config.comWeight = d; }),
      mc_rtc::gui::ArrayInput(
          "CoM stiffness", {"x", "y", "z"}, [&config]() -> Eigen::Vector3d { return config.comStiffness; },
          [&config](const Eigen::Vector3d & d) { config.comStiffness = d; }),
      mc_rtc::gui::ArrayInput(
          "Foot admittance", {"CoPx", "CoPy"}, [&config]() -> Eigen::Vector2d { return config.copAdmittance; },
          [&config](const Eigen::Vector2d & a) { config.copAdmittance = a; }),
      mc_rtc::gui::ArrayInput(
          "Foot CoP lambda", {"CoPx", "CoPy", "Fz"}, [&config]() -> Eigen::Vector3d { return config.copFzLambda; },
          [&config](const Eigen::Vector3d & a) { config.copFzLambda = a; }),
      mc_rtc::gui::NumberInput(
          "CoM height", [&config]() { return config.comHeight; }, [&config](double d) { config.comHeight = d; }),
      mc_rtc::gui::NumberInput(
          "Admittance Delay", [&config]() { return config.delayCoP; }, [&config](double d) { config.delayCoP = d; }),
      mc_rtc::gui::ArrayInput(
          "Foot force difference Admittance", {"Fx", "Fy", "Fz"}, [&config]() -> Eigen::Vector3d
          { return config.dfAdmittance; }, [&config](const Eigen::Vector3d & a) { config.dfAdmittance = a; }),
      mc_rtc::gui::ArrayInput(
          "Foot force difference Damping", {"Fx", "Fy", "Fz"}, [&config]() -> Eigen::Vector3d
          { return config.dfDamping; }, [&config](const Eigen::Vector3d & a) { config.dfDamping = a; }),
      mc_rtc::gui::NumberInput(
          "CoMd Error gain", [&config]() { return config.comdErrorGain; },
          [&config](double a) { config.comdErrorGain = a; }),
      mc_rtc::gui::NumberInput(
          "ZMPd gain", [&config]() { return config.zmpdGain; }, [&config](double a) { config.zmpdGain = a; }),
      mc_rtc::gui::ArrayInput(
          "DCM filters", {"Integrator T [s]", "Derivator T [s]"}, [&config]() -> Eigen::Vector2d
          { return {config.dcmDerivatorTimeConstant, config.dcmIntegratorTimeConstant}; },
          [&config](const Eigen::Vector2d & T)
          {
            config.dcmDerivatorTimeConstant = T(0);
            config.dcmIntegratorTimeConstant = T(1);
          }),
      mc_rtc::gui::NumberInput(
          "Torso pitch [rad]", [&config]() { return config.torsoPitch; },
          [&config](double pitch) { config.torsoPitch = pitch; }));
  category_.pop_back();
  category_.push_back("Advanced");
  gui.addElement(category_,
                 mc_rtc::gui::Checkbox(
                     "Constrain CoP", [&config]() { return config.constrainCoP; },
                     [&config]() { config.constrainCoP = !config.constrainCoP; }),
                 mc_rtc::gui::NumberInput(
                     "Admittance Velocity Filter [0-1]", [&config]() { return config.copVelFilterGain; },
                     [&config](double gain) { config.copVelFilterGain = gain; }),
                 mc_rtc::gui::ArrayInput(
                     "Vertical drift compensation", {"frequency", "stiffness"},
                     [&config]() -> Eigen::Vector2d { return {config.vdcFrequency, config.vdcStiffness}; },
                     [&config](const Eigen::Vector2d & v)
                     {
                       config.vdcFrequency = v(0);
                       config.vdcStiffness = v(1);
                     }),
                 mc_rtc::gui::NumberInput(
                     "Torso pitch [rad]", [&config]() { return config.torsoPitch; },
                     [&config](double pitch)
                     {
                       config.torsoPitch = pitch;
                       ;
                     }),
                 mc_rtc::gui::ArrayInput(
                     "Admittance damping", {"wx", "wy", "wz", "vx", "vy", "vz"},
                     [&config]() -> Eigen::Vector6d { return config.contactDamping.vector(); },
                     [&config](const Eigen::Vector6d & d) { config.contactDamping = sva::MotionVecd(d); }));

  category_.push_back("DCM Bias");
  gui.addElement(category_, mc_rtc::gui::ElementsStacking::Horizontal,
                 mc_rtc::gui::Checkbox(
                     "Enabled", [&config]() { return config.dcmBias.withDCMBias; },
                     [&config]() { config.dcmBias.withDCMBias = !config.dcmBias.withDCMBias; }),
                 mc_rtc::gui::Checkbox(
                     "Correct CoM Pos", [&config]() { return config.dcmBias.correctCoMPos; },
                     [&config]() { config.dcmBias.correctCoMPos = !config.dcmBias.correctCoMPos; }),
                 mc_rtc::gui::Checkbox(
                     "Use Filtered DCM", [&config]() { return config.dcmBias.withDCMFilter; },
                     [&config]() { config.dcmBias.withDCMFilter = !config.dcmBias.withDCMFilter; }));
  gui.addElement(category_,
                 mc_rtc::gui::NumberInput(
                     "dcmMeasureErrorStd", [&config]() { return config.dcmBias.dcmMeasureErrorStd; },
                     [&config](double v) { config.dcmBias.dcmMeasureErrorStd = v; }),
                 mc_rtc::gui::NumberInput(
                     "zmpMeasureErrorStd", [&config]() { return config.dcmBias.zmpMeasureErrorStd; },
                     [&config](double v) { config.dcmBias.zmpMeasureErrorStd = v; }),
                 mc_rtc::gui::NumberInput(
                     "driftPerSecondStd", [&config]() { return config.dcmBias.biasDriftPerSecondStd; },
                     [&config](double v) { config.dcmBias.biasDriftPerSecondStd = v; }),
                 mc_rtc::gui::ArrayInput(
                     "Bias Limit [m]", {"sagital", "lateral"},
                     [&config]() -> const Eigen::Vector2d & { return config.dcmBias.biasLimit; },
                     [&config](const Eigen::Vector2d & v) { config.dcmBias.biasLimit = v; }),
                 mc_rtc::gui::ArrayInput(
                     "CoM bias Limit [m]", {"sagital", "lateral"},
                     [&config]() -> const Eigen::Vector2d & { return config.dcmBias.comBiasLimit; },
                     [&config](const Eigen::Vector2d & v) { config.dcmBias.comBiasLimit = v; }));
  category_.pop_back();
  category_.push_back("Ext Wrench");
  gui.addElement(category_,
                 mc_rtc::gui::Checkbox(
                     "addExpectedCoMOffset", [&config]() { return config.extWrench.addExpectedCoMOffset; },
                     [&config]() { config.extWrench.addExpectedCoMOffset = !config.extWrench.addExpectedCoMOffset; }),
                 mc_rtc::gui::Checkbox(
                     "subtractMeasuredValue", [&config]() { return config.extWrench.subtractMeasuredValue; },
                     [&config]() { config.extWrench.subtractMeasuredValue = !config.extWrench.subtractMeasuredValue; }),
                 mc_rtc::gui::Checkbox(
                     "modifyCoMErr", [&config]() { return config.extWrench.modifyCoMErr; },
                     [&config]() { config.extWrench.modifyCoMErr = !config.extWrench.modifyCoMErr; }),
                 mc_rtc::gui::Checkbox(
                     "modifyZMPErr", [&config]() { return config.extWrench.modifyZMPErr; },
                     [&config]() { config.extWrench.modifyZMPErr = !config.extWrench.modifyZMPErr; }),
                 mc_rtc::gui::Checkbox(
                     "modifyZMPErrD", [&config]() { return config.extWrench.modifyZMPErrD; },
                     [&config]() { config.extWrench.modifyZMPErrD = !config.extWrench.modifyZMPErrD; }),
                 mc_rtc::gui::Checkbox(
                     "excludeFromDCMBiasEst", [&config]() { return config.extWrench.excludeFromDCMBiasEst; },
                     [&config]() { config.extWrench.excludeFromDCMBiasEst = !config.extWrench.excludeFromDCMBiasEst; }),
                 mc_rtc::gui::NumberInput(
                     "Limit of comOffsetErrCoM", [&config]() { return config.extWrench.comOffsetErrCoMLimit; },
                     [&config](double a) { config.extWrench.comOffsetErrCoMLimit = a; }),
                 mc_rtc::gui::NumberInput(
                     "Limit of comOffsetErrZMP", [&config]() { return config.extWrench.comOffsetErrZMPLimit; },
                     [&config](double a) { config.extWrench.comOffsetErrZMPLimit = a; }),
                 mc_rtc::gui::NumberInput(
                     "Cutoff period of extWrenchSumLowPass",
                     [&config]() { return config.extWrench.extWrenchSumLowPassCutoffPeriod; },
                     [&config](double a) { config.extWrench.extWrenchSumLowPassCutoffPeriod = a; }),
                 mc_rtc::gui::NumberInput(
                     "Cutoff period of comOffsetLowPass",
                     [&config]() { return config.extWrench.comOffsetLowPassCutoffPeriod; },
                     [&config](double a) { config.extWrench.comOffsetLowPassCutoffPeriod = a; }),
                 mc_rtc::gui::NumberInput(
                     "Cutoff period of comOffsetLowPassCoM",
                     [&config]() { return config.extWrench.comOffsetLowPassCoMCutoffPeriod; },
                     [&config](double a) { config.extWrench.comOffsetLowPassCoMCutoffPeriod = a; }),
                 mc_rtc::gui::NumberInput(
                     "Time constant of comOffsetDerivator",
                     [&config]() { return config.extWrench.comOffsetDerivatorTimeConstant; },
                     [&config](double a) { config.extWrench.comOffsetDerivatorTimeConstant = a; }));
}

} // namespace mc_rbdyn::gui::lipm_stabilizer
