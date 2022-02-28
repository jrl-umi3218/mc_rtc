/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 */

#include <mc_filter/utils/clamp.h>
#include <mc_rtc/gui.h>
#include <mc_tasks/lipm_stabilizer/StabilizerTask.h>

namespace mc_tasks
{
namespace lipm_stabilizer
{

using ::mc_filter::utils::clamp;

inline Eigen::Vector2d vecFromError(const Eigen::Vector3d & error)
{
  double x = -std::round(error.x() * 1000.);
  double y = -std::round(error.y() * 1000.);
  return Eigen::Vector2d{x, y};
}

void StabilizerTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  using namespace mc_rtc::gui;
  using Style = mc_rtc::gui::plot::Style;

  // clang-format off
  auto addConfigButtons =
    [this,&gui](const std::vector<std::string> & category)
    {
      gui.addElement(category, ElementsStacking::Horizontal,
                     Button("Enable", [this]() { enable(); }),
                     Button("Reconfigure", [this]() { reconfigure(); }),
                     Button("Commit", [this]() { commitConfig(); }));
    };
  // clang-format on

  gui.addElement({"Tasks", name_, "Main"}, Button("Disable", [this]() { disable(); }),
                 Button("Reset DCM integrator", [this]() { dcmIntegrator_.reset(Eigen::Vector3d::Zero()); }));
  addConfigButtons({"Tasks", name_, "Main"});
  gui.addElement(
      {"Tasks", name_, "Main"},
      ArrayInput(
          "Foot admittance", {"CoPx", "CoPy"},
          [this]() -> Eigen::Vector2d {
            return {c_.copAdmittance.x(), c_.copAdmittance.y()};
          },
          [this](const Eigen::Vector2d & a) { copAdmittance(a); }),
      ArrayInput(
          "Foot force difference", {"Admittance", "Damping"},
          [this]() -> Eigen::Vector2d {
            return {c_.dfzAdmittance, c_.dfzDamping};
          },
          [this](const Eigen::Vector2d & a) {
            dfzAdmittance(a(0));
            dfzDamping(a(1));
          }),
      ArrayInput(
          "DCM gains", {"Prop.", "Integral", "Deriv."},
          [this]() -> Eigen::Vector3d {
            return {c_.dcmPropGain, c_.dcmIntegralGain, c_.dcmDerivGain};
          },
          [this](const Eigen::Vector3d & gains) { dcmGains(gains(0), gains(1), gains(2)); }),
      NumberInput(
          "CoMd Error gain", [this]() { return c_.comdErrorGain; }, [this](double a) { c_.comdErrorGain = a; }),
      NumberInput(
          "ZMPd gain", [this]() { return c_.zmpdGain; }, [this](double a) { c_.zmpdGain = a; }),
      ArrayInput(
          "DCM filters", {"Integrator T [s]", "Derivator T [s]"},
          [this]() -> Eigen::Vector2d {
            return {dcmIntegrator_.timeConstant(), dcmDerivator_.timeConstant()};
          },
          [this](const Eigen::Vector2d & T) {
            dcmIntegratorTimeConstant(T(0));
            dcmDerivatorTimeConstant(T(1));
          }));
  gui.addElement({"Tasks", name_, "Advanced"}, Button("Disable", [this]() { disable(); }));
  addConfigButtons({"Tasks", name_, "Advanced"});
  gui.addElement({"Tasks", name_, "Advanced"},
                 Checkbox(
                     "Apply CoM admittance only in double support?", [this]() { return zmpccOnlyDS_; },
                     [this]() { zmpccOnlyDS_ = !zmpccOnlyDS_; }));
  zmpcc_.addToGUI(gui, {"Tasks", name_, "Advanced"});
  gui.addElement(
      {"Tasks", name_, "Advanced"},
      NumberInput(
          "Admittance Velocity Filter [0-1]", [this]() { return c_.copVelFilterGain; },
          [this](double gain) { copVelFilterGain(gain); }),
      ArrayInput(
          "Max cop angular velocity [rad/s]",
          [this]() -> const Eigen::Vector3d & { return footTasks.at(ContactState::Left)->maxAngularVel(); },
          [this](const Eigen::Vector3d & v) {
            footTasks.at(ContactState::Left)->maxAngularVel(v);
            footTasks.at(ContactState::Right)->maxAngularVel(v);
          }),
      ArrayInput(
          "Max cop linear velocity [m/s]",
          [this]() -> const Eigen::Vector3d & { return footTasks.at(ContactState::Left)->maxLinearVel(); },
          [this](const Eigen::Vector3d & v) {
            footTasks.at(ContactState::Left)->maxLinearVel(v);
            footTasks.at(ContactState::Right)->maxLinearVel(v);
          }),
      ArrayInput(
          "Vertical drift compensation", {"frequency", "stiffness"},
          [this]() -> Eigen::Vector2d {
            return {c_.vdcFrequency, c_.vdcStiffness};
          },
          [this](const Eigen::Vector2d & v) {
            vdcFrequency(v(0));
            vdcStiffness(v(1));
          }),
      NumberInput(
          "Torso pitch [rad]", [this]() { return c_.torsoPitch; }, [this](double pitch) { torsoPitch(pitch); }));
  gui.addElement({"Tasks", name_, "Advanced", "DCM Bias"}, mc_rtc::gui::ElementsStacking::Horizontal,
                 Checkbox(
                     "Enabled", [this]() { return c_.dcmBias.withDCMBias; },
                     [this]() { c_.dcmBias.withDCMBias = !c_.dcmBias.withDCMBias; }),
                 Checkbox(
                     "Correct CoM Pos", [this]() { return c_.dcmBias.correctCoMPos; },
                     [this]() { c_.dcmBias.correctCoMPos = !c_.dcmBias.correctCoMPos; }),
                 Checkbox(
                     "Use Filtered DCM", [this]() { return c_.dcmBias.withDCMFilter; },
                     [this]() { c_.dcmBias.withDCMFilter = !c_.dcmBias.withDCMFilter; }));
  gui.addElement({"Tasks", name_, "Advanced", "DCM Bias"},
                 NumberInput(
                     "dcmMeasureErrorStd", [this]() { return c_.dcmBias.dcmMeasureErrorStd; },
                     [this](double v) {
                       c_.dcmBias.dcmMeasureErrorStd = v;
                       dcmEstimator_.setDcmMeasureErrorStd(v);
                     }),
                 NumberInput(
                     "zmpMeasureErrorStd", [this]() { return c_.dcmBias.zmpMeasureErrorStd; },
                     [this](double v) {
                       c_.dcmBias.zmpMeasureErrorStd = v;
                       dcmEstimator_.setZmpMeasureErrorStd(v);
                     }),
                 NumberInput(
                     "driftPerSecondStd", [this]() { return c_.dcmBias.biasDriftPerSecondStd; },
                     [this](double v) {
                       c_.dcmBias.biasDriftPerSecondStd = v;
                       dcmEstimator_.setBiasDriftPerSecond(v);
                     }),
                 ArrayInput(
                     "Bias Limit [m]", {"sagital", "lateral"},
                     [this]() -> const Eigen::Vector2d & { return c_.dcmBias.biasLimit; },
                     [this](const Eigen::Vector2d & v) {
                       c_.dcmBias.biasLimit = v;
                       dcmEstimator_.setBiasLimit(v);
                     }),
                 ArrayInput(
                     "CoM bias Limit [m]", {"sagital", "lateral"},
                     [this]() -> const Eigen::Vector2d & { return c_.dcmBias.comBiasLimit; },
                     [this](const Eigen::Vector2d & v) { c_.dcmBias.comBiasLimit = v; }),
                 ArrayLabel("Local Bias", [this]() { return dcmEstimator_.getLocalBias(); }));
  gui.addElement({"Tasks", name_, "Advanced", "Ext Wrench"},
                 Checkbox(
                     "addExpectedCoMOffset", [this]() { return c_.extWrench.addExpectedCoMOffset; },
                     [this]() { c_.extWrench.addExpectedCoMOffset = !c_.extWrench.addExpectedCoMOffset; }),
                 Checkbox(
                     "subtractMeasuredValue", [this]() { return c_.extWrench.subtractMeasuredValue; },
                     [this]() { c_.extWrench.subtractMeasuredValue = !c_.extWrench.subtractMeasuredValue; }),
                 Checkbox(
                     "modifyCoMErr", [this]() { return c_.extWrench.modifyCoMErr; },
                     [this]() { c_.extWrench.modifyCoMErr = !c_.extWrench.modifyCoMErr; }),
                 Checkbox(
                     "modifyZMPErr", [this]() { return c_.extWrench.modifyZMPErr; },
                     [this]() { c_.extWrench.modifyZMPErr = !c_.extWrench.modifyZMPErr; }),
                 Checkbox(
                     "modifyZMPErrD", [this]() { return c_.extWrench.modifyZMPErrD; },
                     [this]() { c_.extWrench.modifyZMPErrD = !c_.extWrench.modifyZMPErrD; }),
                 NumberInput(
                     "Limit of comOffsetErrCoM", [this]() { return c_.extWrench.comOffsetErrCoMLimit; },
                     [this](double a) { c_.extWrench.comOffsetErrCoMLimit = a; }),
                 NumberInput(
                     "Limit of comOffsetErrZMP", [this]() { return c_.extWrench.comOffsetErrZMPLimit; },
                     [this](double a) { c_.extWrench.comOffsetErrZMPLimit = a; }),
                 NumberInput(
                     "Cutoff period of extWrenchSumLowPass", [this]() { return extWrenchSumLowPass_.cutoffPeriod(); },
                     [this](double a) { extWrenchSumLowPassCutoffPeriod(a); }),
                 NumberInput(
                     "Cutoff period of comOffsetLowPass", [this]() { return comOffsetLowPass_.cutoffPeriod(); },
                     [this](double a) { comOffsetLowPassCutoffPeriod(a); }),
                 NumberInput(
                     "Cutoff period of comOffsetLowPassCoM", [this]() { return comOffsetLowPassCoM_.cutoffPeriod(); },
                     [this](double a) { comOffsetLowPassCoMCutoffPeriod(a); }),
                 NumberInput(
                     "Time constant of comOffsetDerivator", [this]() { return comOffsetDerivator_.timeConstant(); },
                     [this](double a) { comOffsetDerivatorTimeConstant(a); }));

  gui.addElement({"Tasks", name_, "Debug"}, Button("Disable", [this]() { disable(); }));
  addConfigButtons({"Tasks", name_, "Debug"});
  gui.addElement({"Tasks", name_, "Debug"}, Button("Dump configuration", [this]() {
                   mc_rtc::log::info("[LIPMStabilizerTask] configuration (YAML)");
                   mc_rtc::log::info(c_.save().dump(true, true));
                 }));

  gui.addElement({"Tasks", name_, "Debug"}, ElementsStacking::Horizontal,
                 Button("Plot DCM-ZMP Tracking (x)",
                        [this, &gui]() {
                          gui.addPlot(
                              "DCM-ZMP Tracking (x)", plot::X("t", [this]() { return t_; }),
                              plot::Y(
                                  "support_min", [this]() { return supportMin_.x(); }, Color::Red),
                              plot::Y(
                                  "support_max", [this]() { return supportMax_.x(); }, Color::Red),
                              plot::Y(
                                  "dcm_ref", [this]() { return dcmTarget_.x(); }, Color::Red),
                              plot::Y(
                                  "dcm_mes", [this]() { return measuredDCM_.x(); }, Color::Magenta, Style::Dashed),
                              plot::Y(
                                  "dcm_unbiased", [this]() { return measuredDCMUnbiased_.x(); }, Color::Magenta),
                              plot::Y(
                                  "zmp_ref", [this]() { return zmpTarget_.x(); }, Color::Cyan),
                              plot::Y(
                                  "zmp_mes", [this]() { return measuredZMP_.x(); }, Color::Blue, Style::Dashed),
                              plot::Y(
                                  "zmp_stabi", [this]() { return distribZMP_.x(); }, Color::Blue));
                        }),
                 Button("Stop DCM-ZMP (x)", [&gui]() { gui.removePlot("DCM-ZMP Tracking (x)"); }));

  gui.addElement({"Tasks", name_, "Debug"}, ElementsStacking::Horizontal,
                 Button("Plot DCM-ZMP Tracking (y)",
                        [this, &gui]() {
                          gui.addPlot(
                              "DCM-ZMP Tracking (y)", plot::X("t", [this]() { return t_; }),
                              plot::Y(
                                  "support_min", [this]() { return supportMin_.y(); }, Color::Red),
                              plot::Y(
                                  "support_max", [this]() { return supportMax_.y(); }, Color::Red),
                              plot::Y(
                                  "dcm_ref", [this]() { return dcmTarget_.y(); }, Color::Red),
                              plot::Y(
                                  "dcm_mes", [this]() { return measuredDCM_.y(); }, Color::Magenta, Style::Dashed),
                              plot::Y(
                                  "dcm_unbiased", [this]() { return measuredDCMUnbiased_.y(); }, Color::Magenta),
                              plot::Y(
                                  "zmp_ref", [this]() { return zmpTarget_.y(); }, Color::Cyan),
                              plot::Y(
                                  "zmp_mes", [this]() { return measuredZMP_.y(); }, Color::Blue, Style::Dashed),
                              plot::Y(
                                  "zmp_stabi", [this]() { return distribZMP_.y(); }, Color::Blue));
                        }),
                 Button("Stop DCM-ZMP (y)", [&gui]() { gui.removePlot("DCM-ZMP Tracking (y)"); }));

  gui.addElement({"Tasks", name_, "Debug"}, ElementsStacking::Horizontal,
                 Button("Plot CoM Tracking (x)",
                        [this, &gui]() {
                          gui.addPlot("CoM Tracking (x)", plot::X("t", [this]() { return t_; }),
                                      plot::Y(
                                          "com_ref", [this]() { return comTarget_.x(); }, Color::Red),
                                      plot::Y(
                                          "com_mes", [this]() { return measuredCoM_.x(); }, Color::Magenta));
                        }),
                 Button("Stop CoM (x)", [&gui]() { gui.removePlot("CoM Tracking (x)"); }));
  gui.addElement({"Tasks", name_, "Debug"}, ElementsStacking::Horizontal,
                 Button("Plot CoM Tracking (y)",
                        [this, &gui]() {
                          gui.addPlot("CoM Tracking (y)", plot::X("t", [this]() { return t_; }),
                                      plot::Y(
                                          "com_ref", [this]() { return comTarget_.y(); }, Color::Red),
                                      plot::Y(
                                          "com_mes", [this]() { return measuredCoM_.y(); }, Color::Magenta));
                        }),
                 Button("Stop CoM (y)", [&gui]() { gui.removePlot("CoM Tracking (y)"); }));

  gui.addElement({"Tasks", name_, "Debug"}, ElementsStacking::Horizontal,
                 Button("Plot DCM Integrator",
                        [this, &gui]() {
                          gui.addPlot("DCM Integrator", plot::X("t", [this]() { return t_; }),
                                      plot::Y(
                                          "x", [this]() { return dcmIntegrator_.eval().x(); }, Color::Red),
                                      plot::Y(
                                          "y", [this]() { return dcmIntegrator_.eval().y(); }, Color::Green),
                                      plot::Y(
                                          "z", [this]() { return dcmIntegrator_.eval().z(); }, Color::Blue));
                        }),
                 Button("Stop DCM Integrator", [&gui]() { gui.removePlot("DCM Integrator"); }));
  gui.addElement({"Tasks", name_, "Debug"}, ElementsStacking::Horizontal,
                 Button("Plot DCM Derivator",
                        [this, &gui]() {
                          gui.addPlot("DCM Derivator", plot::X("t", [this]() { return t_; }),
                                      plot::Y(
                                          "x", [this]() { return dcmDerivator_.eval().x(); }, Color::Red),
                                      plot::Y(
                                          "y", [this]() { return dcmDerivator_.eval().y(); }, Color::Green),
                                      plot::Y(
                                          "z", [this]() { return dcmDerivator_.eval().z(); }, Color::Blue));
                        }),
                 Button("Stop DCM Derivator", [&gui]() { gui.removePlot("DCM Derivator"); }));

  gui.addElement({"Tasks", name_, "Debug"},
                 ArrayLabel("DCM average error [mm]", {"x", "y"}, [this]() { return vecFromError(dcmAverageError_); }),
                 ArrayLabel("DCM error [mm]", {"x", "y"}, [this]() { return vecFromError(dcmError_); }),
                 ArrayLabel("Foot force difference error [mm]", {"force", "height"}, [this]() {
                   Eigen::Vector3d dfzError = {dfzForceError_, dfzHeightError_, 0.};
                   return vecFromError(dfzError);
                 }));

  ///// GUI MARKERS
  constexpr double ARROW_HEAD_DIAM = 0.015;
  constexpr double ARROW_HEAD_LEN = 0.05;
  constexpr double ARROW_SHAFT_DIAM = 0.015;
  constexpr double FORCE_SCALE = 0.0015;

  ArrowConfig pendulumArrowConfig;
  pendulumArrowConfig.color = Color::Yellow;
  pendulumArrowConfig.end_point_scale = 0.02;
  pendulumArrowConfig.head_diam = .1 * ARROW_HEAD_DIAM;
  pendulumArrowConfig.head_len = .1 * ARROW_HEAD_LEN;
  pendulumArrowConfig.scale = 1.;
  pendulumArrowConfig.shaft_diam = .1 * ARROW_SHAFT_DIAM;
  pendulumArrowConfig.start_point_scale = 0.02;

  ArrowConfig pendulumForceArrowConfig;
  pendulumForceArrowConfig.shaft_diam = 1 * ARROW_SHAFT_DIAM;
  pendulumForceArrowConfig.head_diam = 1 * ARROW_HEAD_DIAM;
  pendulumForceArrowConfig.head_len = 1 * ARROW_HEAD_LEN;
  pendulumForceArrowConfig.scale = 1.;
  pendulumForceArrowConfig.start_point_scale = 0.02;
  pendulumForceArrowConfig.end_point_scale = 0.;

  ArrowConfig netWrenchForceArrowConfig = pendulumForceArrowConfig;
  netWrenchForceArrowConfig.color = Color::Red;

  ArrowConfig refPendulumForceArrowConfig = pendulumForceArrowConfig;
  refPendulumForceArrowConfig = Color::Yellow;

  ForceConfig copForceConfig(Color::Green);
  copForceConfig.start_point_scale = 0.02;
  copForceConfig.end_point_scale = 0.;

  constexpr double COM_POINT_SIZE = 0.02;
  constexpr double DCM_POINT_SIZE = 0.015;

  gui.addElement({"Tasks", name_, "Markers", "CoM-DCM"},
                 Arrow(
                     "Pendulum_CoM", pendulumArrowConfig, [this]() -> Eigen::Vector3d { return zmpTarget_; },
                     [this]() -> Eigen::Vector3d { return comTarget_; }),
                 Point3D("Measured_CoM", PointConfig(Color::Green, COM_POINT_SIZE), [this]() { return measuredCoM_; }),
                 Point3D("Pendulum_DCM", PointConfig(Color::Yellow, DCM_POINT_SIZE), [this]() { return dcmTarget_; }),
                 Point3D("Measured_DCM", PointConfig(Color::Green, DCM_POINT_SIZE),
                         [this]() -> Eigen::Vector3d { return measuredCoM_ + measuredCoMd_ / omega_; }));

  gui.addElement(
      {"Tasks", name_, "Markers", "Net wrench"},
      Point3D("Measured_ZMP", PointConfig(Color::Red, 0.02), [this]() -> Eigen::Vector3d { return measuredZMP_; }),
      Arrow(
          "Measured_ZMPForce", netWrenchForceArrowConfig, [this]() -> Eigen::Vector3d { return measuredZMP_; },
          [this, FORCE_SCALE]() -> Eigen::Vector3d {
            return measuredZMP_ + FORCE_SCALE * measuredNetWrench_.force();
          }));

  for(const auto footTask : footTasks)
  {
    auto footT = footTask.second;
    gui.addElement({"Tasks", name_, "Markers", "Foot wrenches"},
                   Point3D("Stabilizer_" + footT->surface() + "CoP", PointConfig(Color::Magenta, 0.01),
                           [footT]() { return footT->targetCoPW(); }),
                   Force(
                       "Measured_" + footT->surface() + "CoPForce", copForceConfig,
                       [footT, this]() {
                         return robot().indirectSurfaceForceSensor(footT->surface()).worldWrenchWithoutGravity(robot());
                       },
                       [footT]() { return sva::PTransformd(footT->measuredCoPW()); }));
  }

  gui.addElement({"Tasks", name_, "Markers", "Contacts"},
                 Polygon("SupportContacts", Color::Green, [this]() { return supportPolygons_; }));
}

void StabilizerTask::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  MetaTask::removeFromGUI(gui);
  gui.removePlot("DCM-ZMP Tracking (x)");
  gui.removePlot("DCM-ZMP Tracking (y)");
  gui.removePlot("CoM Tracking (x)");
  gui.removePlot("DCM Integrator");
}

void StabilizerTask::addToLogger(mc_rtc::Logger & logger)
{
  // Globbal log entries added to other categories
  MC_RTC_LOG_HELPER("perf_" + name_, runTime_);

  MC_RTC_LOG_HELPER(name_ + "_error_dcm_average", dcmAverageError_);
  MC_RTC_LOG_HELPER(name_ + "_error_dcm_pos", dcmError_);
  MC_RTC_LOG_HELPER(name_ + "_error_dcm_vel", dcmVelError_);
  MC_RTC_LOG_HELPER(name_ + "_error_dfz_force", dfzForceError_);
  MC_RTC_LOG_HELPER(name_ + "_error_dfz_height", dfzHeightError_);
  MC_RTC_LOG_HELPER(name_ + "_error_vdc", vdcHeightError_);
  logger.addLogEntry(name_ + "_admittance_cop", this, [this]() -> const Eigen::Vector2d & { return c_.copAdmittance; });
  logger.addLogEntry(name_ + "_admittance_dfz", this, [this]() { return c_.dfzAdmittance; });
  logger.addLogEntry(name_ + "_dcmDerivator_filtered", this, [this]() { return dcmDerivator_.eval(); });
  logger.addLogEntry(name_ + "_dcmDerivator_timeConstant", this, [this]() { return dcmDerivator_.timeConstant(); });
  logger.addLogEntry(name_ + "_dcmIntegrator_timeConstant", this, [this]() { return dcmIntegrator_.timeConstant(); });
  logger.addLogEntry(name_ + "_dcmTracking_derivGain", this, [this]() { return c_.dcmDerivGain; });
  logger.addLogEntry(name_ + "_dcmTracking_integralGain", this, [this]() { return c_.dcmIntegralGain; });
  logger.addLogEntry(name_ + "_dcmTracking_propGain", this, [this]() { return c_.dcmPropGain; });
  logger.addLogEntry(name_ + "_dcmTracking_comdErrorGain", this, [this]() { return c_.comdErrorGain; });
  logger.addLogEntry(name_ + "_dcmTracking_zmpdGain", this, [this]() { return c_.zmpdGain; });
  logger.addLogEntry(name_ + "_dcmBias_dcmMeasureErrorStd", this, [this]() { return c_.dcmBias.dcmMeasureErrorStd; });
  logger.addLogEntry(name_ + "_dcmBias_zmpMeasureErrorStd", this, [this]() { return c_.dcmBias.zmpMeasureErrorStd; });
  logger.addLogEntry(name_ + "_dcmBias_driftPerSecondStd", this, [this]() { return c_.dcmBias.biasDriftPerSecondStd; });
  logger.addLogEntry(name_ + "_dcmBias_biasLimit", this,
                     [this]() -> const Eigen::Vector2d & { return c_.dcmBias.biasLimit; });
  logger.addLogEntry(name_ + "_dcmBias_localBias", this, [this]() { return dcmEstimator_.getLocalBias(); });
  logger.addLogEntry(name_ + "_dcmBias_bias", this, [this]() { return dcmEstimator_.getBias(); });
  MC_RTC_LOG_HELPER(name_ + "_extWrench_comOffsetTarget", comOffsetTarget_);
  MC_RTC_LOG_HELPER(name_ + "_extWrench_comOffsetMeasured", comOffsetMeasured_);
  MC_RTC_LOG_HELPER(name_ + "_extWrench_comOffsetErr", comOffsetErr_);
  MC_RTC_LOG_HELPER(name_ + "_extWrench_comOffsetErr_CoM", comOffsetErrCoM_);
  MC_RTC_LOG_HELPER(name_ + "_extWrench_comOffsetErr_ZMP", comOffsetErrZMP_);
  logger.addLogEntry(name_ + "_extWrench_comOffsetErr_CoMLimit", this,
                     [this]() { return c_.extWrench.comOffsetErrCoMLimit; });
  logger.addLogEntry(name_ + "_extWrench_comOffsetErr_ZMPLimit", this,
                     [this]() { return c_.extWrench.comOffsetErrZMPLimit; });
  logger.addLogEntry(name_ + "_extWrench_comOffsetDerivator", this, [this]() { return comOffsetDerivator_.eval(); });
  logger.addLogEntry(name_ + "_dfz_damping", this, [this]() { return c_.dfzDamping; });
  logger.addLogEntry(name_ + "_fdqp_weights_ankleTorque", this,
                     [this]() { return std::pow(c_.fdqpWeights.ankleTorqueSqrt, 2); });
  logger.addLogEntry(name_ + "_fdqp_weights_netWrench", this,
                     [this]() { return std::pow(c_.fdqpWeights.netWrenchSqrt, 2); });
  logger.addLogEntry(name_ + "_fdqp_weights_pressure", this,
                     [this]() { return std::pow(c_.fdqpWeights.pressureSqrt, 2); });
  logger.addLogEntry(name_ + "_vdc_frequency", this, [this]() { return c_.vdcFrequency; });
  logger.addLogEntry(name_ + "_vdc_stiffness", this, [this]() { return c_.vdcStiffness; });
  MC_RTC_LOG_HELPER(name_ + "_desired_wrench", desiredWrench_);
  MC_RTC_LOG_HELPER(name_ + "_wrench", distribWrench_);
  MC_RTC_LOG_HELPER(name_ + "_support_min", supportMin_);
  MC_RTC_LOG_HELPER(name_ + "_support_max", supportMax_);
  MC_RTC_LOG_HELPER(name_ + "_left_foot_ratio", leftFootRatio_);

  // Stabilizer targets
  MC_RTC_LOG_HELPER(name_ + "_target_pendulum_com", comTarget_);
  MC_RTC_LOG_HELPER(name_ + "_target_pendulum_comd", comdTarget_);
  MC_RTC_LOG_HELPER(name_ + "_target_pendulum_comdd", comddTarget_);
  MC_RTC_LOG_HELPER(name_ + "_target_pendulum_dcm", dcmTarget_);
  MC_RTC_LOG_HELPER(name_ + "_target_pendulum_omega", omega_);
  MC_RTC_LOG_HELPER(name_ + "_target_pendulum_zmp", zmpTarget_);
  MC_RTC_LOG_HELPER(name_ + "_target_pendulum_zmpd", zmpdTarget_);
  MC_RTC_LOG_HELPER(name_ + "_target_stabilizer_zmp", distribZMP_);

  logger.addLogEntry(name_ + "_contactState", this, [this]() -> int {
    if(inDoubleSupport())
      return 0;
    else if(inContact(ContactState::Left))
      return 1;
    else if(inContact(ContactState::Right))
      return -1;
    else
      return -3;
  });

  // Log computed robot properties
  logger.addLogEntry(name_ + "_controlRobot_LeftFoot", this,
                     [this]() { return robot().surfacePose(footSurface(ContactState::Left)); });
  logger.addLogEntry(name_ + "_controlRobot_RightFoot", this,
                     [this]() { return robot().surfacePose(footSurface(ContactState::Right)); });
  logger.addLogEntry(name_ + "_controlRobot_com", this, [this]() { return robot().com(); });
  logger.addLogEntry(name_ + "_controlRobot_comd", this, [this]() { return robot().comVelocity(); });
  logger.addLogEntry(name_ + "_controlRobot_posW", this,
                     [this]() -> const sva::PTransformd & { return robot().posW(); });

  logger.addLogEntry(name_ + "_realRobot_LeftFoot", this,
                     [this]() { return realRobot().surfacePose(footSurface(ContactState::Left)); });
  logger.addLogEntry(name_ + "_realRobot_RightFoot", this,
                     [this]() { return realRobot().surfacePose(footSurface(ContactState::Right)); });
  MC_RTC_LOG_HELPER(name_ + "_realRobot_com", measuredCoM_);
  MC_RTC_LOG_HELPER(name_ + "_realRobot_comd", measuredCoMd_);
  MC_RTC_LOG_HELPER(name_ + "_realRobot_dcm", measuredDCM_);
  MC_RTC_LOG_HELPER(name_ + "_realRobot_dcm_unbiased", measuredDCMUnbiased_);
  MC_RTC_LOG_HELPER(name_ + "_realRobot_com_unbiased", measuredCoMUnbiased_);
  logger.addLogEntry(name_ + "_realRobot_posW", this,
                     [this]() -> const sva::PTransformd & { return realRobot().posW(); });
  logger.addLogEntry(name_ + "_realRobot_wrench", this,
                     [this]() -> const sva::ForceVecd & { return measuredNetWrench_; });
  logger.addLogEntry(name_ + "_realRobot_zmp", this, [this]() -> const Eigen::Vector3d & { return measuredZMP_; });

  zmpcc_.addToLogger(logger, name_);

  MetaTask::addToLogger(*comTask, logger);
  MetaTask::addToLogger(*pelvisTask, logger);
  MetaTask::addToLogger(*torsoTask, logger);
}

void StabilizerTask::removeFromLogger(mc_rtc::Logger & logger)
{
  MetaTask::removeFromLogger(logger);
  zmpcc_.removeFromLogger(logger, name_);
  MetaTask::removeFromLogger(*comTask, logger);
  MetaTask::removeFromLogger(*pelvisTask, logger);
  MetaTask::removeFromLogger(*torsoTask, logger);
  for(const auto & footT : contactTasks)
  {
    MetaTask::removeFromLogger(*footT, logger);
  }
}

} // namespace lipm_stabilizer
} // namespace mc_tasks
