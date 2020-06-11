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
  gui.addElement({"Tasks", name_, "Main"},
                 ArrayInput("Foot admittance", {"CoPx", "CoPy"},
                            [this]() -> Eigen::Vector2d {
                              return {c_.copAdmittance.x(), c_.copAdmittance.y()};
                            },
                            [this](const Eigen::Vector2d & a) { c_.copAdmittance = clamp(a, 0., MAX_COP_ADMITTANCE); }),
                 ArrayInput("Foot force difference", {"Admittance", "Damping"},
                            [this]() -> Eigen::Vector2d {
                              return {c_.dfzAdmittance, c_.dfzDamping};
                            },
                            [this](const Eigen::Vector2d & a) {
                              c_.dfzAdmittance = clamp(a(0), 0., MAX_DFZ_ADMITTANCE);
                              c_.dfzDamping = clamp(a(1), 0., MAX_DFZ_DAMPING);
                            }),
                 ArrayInput("DCM gains", {"Prop.", "Integral", "Deriv."},
                            [this]() -> Eigen::Vector3d {
                              return {c_.dcmPropGain, c_.dcmIntegralGain, c_.dcmDerivGain};
                            },
                            [this](const Eigen::Vector3d & gains) {
                              c_.dcmPropGain = clamp(gains(0), 0., MAX_DCM_P_GAIN);
                              c_.dcmIntegralGain = clamp(gains(1), 0., MAX_DCM_I_GAIN);
                              c_.dcmDerivGain = clamp(gains(2), 0., MAX_DCM_D_GAIN);
                            }),
                 ArrayInput("DCM filters", {"Integrator T [s]", "Derivator T [s]"},
                            [this]() -> Eigen::Vector2d {
                              return {dcmIntegrator_.timeConstant(), dcmDerivator_.timeConstant()};
                            },
                            [this](const Eigen::Vector2d & T) {
                              dcmIntegrator_.timeConstant(T(0));
                              dcmDerivator_.timeConstant(T(1));
                            }));
  gui.addElement({"Tasks", name_, "Advanced"}, Button("Disable", [this]() { disable(); }));
  addConfigButtons({"Tasks", name_, "Advanced"});
  gui.addElement({"Tasks", name_, "Advanced"},
                 Checkbox("Apply CoM admittance only in double support?", [this]() { return zmpccOnlyDS_; },
                          [this]() { zmpccOnlyDS_ = !zmpccOnlyDS_; }));
  zmpcc_.addToGUI(gui, {"Tasks", name_, "Advanced"});
  gui.addElement({"Tasks", name_, "Advanced"},
                 ArrayInput("Vertical drift compensation", {"frequency", "stiffness"},
                            [this]() -> Eigen::Vector2d {
                              return {c_.vdcFrequency, c_.vdcStiffness};
                            },
                            [this](const Eigen::Vector2d & v) {
                              c_.vdcFrequency = clamp(v(0), 0., 10.);
                              c_.vdcStiffness = clamp(v(1), 0., 1e4);
                            }),
                 NumberInput("Torso pitch [rad]", [this]() { return c_.torsoPitch; },
                             [this](double pitch) { c_.torsoPitch = pitch; }));

  gui.addElement({"Tasks", name_, "Debug"}, Button("Disable", [this]() { disable(); }));
  addConfigButtons({"Tasks", name_, "Debug"});
  gui.addElement({"Tasks", name_, "Debug"}, Button("Dump configuration", [this]() {
                   mc_rtc::log::info("[LIPMStabilizerTask] configuration (YAML)");
                   mc_rtc::log::info(c_.save().dump(true, true));
                 }));

  gui.addElement({"Tasks", name_, "Debug"}, ElementsStacking::Horizontal,
                 Button("Plot DCM-ZMP Tracking (x)",
                        [this, &gui]() {
                          gui.addPlot("DCM-ZMP Tracking (x)", plot::X("t", [this]() { return t_; }),
                                      plot::Y("support_min", [this]() { return supportMin_.x(); }, Color::Red),
                                      plot::Y("support_max", [this]() { return supportMax_.x(); }, Color::Red),
                                      plot::Y("dcm_ref", [this]() { return dcmTarget_.x(); }, Color::Red),
                                      plot::Y("dcm_mes", [this]() { return measuredDCM_.x(); }, Color::Magenta),
                                      plot::Y("zmp_ref", [this]() { return zmpTarget_.x(); }, Color::Blue),
                                      plot::Y("zmp_mes", [this]() { return measuredZMP_.x(); }, Color::Cyan));
                        }),
                 Button("Stop DCM-ZMP (x)", [&gui]() { gui.removePlot("DCM-ZMP Tracking (x)"); }));

  gui.addElement({"Tasks", name_, "Debug"}, ElementsStacking::Horizontal,
                 Button("Plot DCM-ZMP Tracking (y)",
                        [this, &gui]() {
                          gui.addPlot("DCM-ZMP Tracking (y)", plot::X("t", [this]() { return t_; }),
                                      plot::Y("support_min", [this]() { return supportMin_.y(); }, Color::Red),
                                      plot::Y("support_max", [this]() { return supportMax_.y(); }, Color::Red),
                                      plot::Y("dcm_ref", [this]() { return dcmTarget_.y(); }, Color::Red),
                                      plot::Y("dcm_mes", [this]() { return measuredDCM_.y(); }, Color::Magenta),
                                      plot::Y("zmp_ref", [this]() { return zmpTarget_.y(); }, Color::Blue),
                                      plot::Y("zmp_mes", [this]() { return measuredZMP_.y(); }, Color::Cyan));
                        }),
                 Button("Stop DCM-ZMP (y)", [&gui]() { gui.removePlot("DCM-ZMP Tracking (y)"); }));

  gui.addElement({"Tasks", name_, "Debug"}, ElementsStacking::Horizontal,
                 Button("Plot CoM Tracking (x)",
                        [this, &gui]() {
                          gui.addPlot("CoM Tracking (x)", plot::X("t", [this]() { return t_; }),
                                      plot::Y("com_ref", [this]() { return comTarget_.x(); }, Color::Red),
                                      plot::Y("com_mes", [this]() { return measuredCoM_.y(); }, Color::Magenta));
                        }),
                 Button("Stop CoM (x)", [&gui]() { gui.removePlot("CoM Tracking (x)"); }));

  gui.addElement({"Tasks", name_, "Debug"}, ElementsStacking::Horizontal,
                 Button("Plot DCM Integrator",
                        [this, &gui]() {
                          gui.addPlot("DCM Integrator", plot::X("t", [this]() { return t_; }),
                                      plot::Y("x", [this]() { return dcmIntegrator_.eval().x(); }, Color::Red),
                                      plot::Y("y", [this]() { return dcmIntegrator_.eval().y(); }, Color::Green),
                                      plot::Y("z", [this]() { return dcmIntegrator_.eval().z(); }, Color::Blue));
                        }),
                 Button("Stop DCM Integrator", [&gui]() { gui.removePlot("DCM Integrator"); }));
  gui.addElement({"Tasks", name_, "Debug"}, ElementsStacking::Horizontal,
                 Button("Plot DCM Derivator",
                        [this, &gui]() {
                          gui.addPlot("DCM Derivator", plot::X("t", [this]() { return t_; }),
                                      plot::Y("x", [this]() { return dcmDerivator_.eval().x(); }, Color::Red),
                                      plot::Y("y", [this]() { return dcmDerivator_.eval().y(); }, Color::Green),
                                      plot::Y("z", [this]() { return dcmDerivator_.eval().z(); }, Color::Blue));
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
                 Arrow("Pendulum_CoM", pendulumArrowConfig, [this]() -> Eigen::Vector3d { return zmpTarget_; },
                       [this]() -> Eigen::Vector3d { return comTarget_; }),
                 Point3D("Measured_CoM", PointConfig(Color::Green, COM_POINT_SIZE), [this]() { return measuredCoM_; }),
                 Point3D("Pendulum_DCM", PointConfig(Color::Yellow, DCM_POINT_SIZE), [this]() { return dcmTarget_; }),
                 Point3D("Measured_DCM", PointConfig(Color::Green, DCM_POINT_SIZE),
                         [this]() -> Eigen::Vector3d { return measuredCoM_ + measuredCoMd_ / omega_; }));

  gui.addElement(
      {"Tasks", name_, "Markers", "Net wrench"},
      Point3D("Measured_ZMP", PointConfig(Color::Red, 0.02), [this]() -> Eigen::Vector3d { return measuredZMP_; }),
      Arrow("Measured_ZMPForce", netWrenchForceArrowConfig, [this]() -> Eigen::Vector3d { return measuredZMP_; },
            [this, FORCE_SCALE]() -> Eigen::Vector3d {
              return measuredZMP_ + FORCE_SCALE * measuredNetWrench_.force();
            }));

  for(const auto footTask : footTasks)
  {
    auto footT = footTask.second;
    gui.addElement({"Tasks", name_, "Markers", "Foot wrenches"},
                   Point3D("Stabilizer_" + footT->surface() + "CoP", PointConfig(Color::Magenta, 0.01),
                           [footT]() { return footT->targetCoPW(); }),
                   Force("Measured_" + footT->surface() + "CoPForce", copForceConfig,
                         [footT]() { return footT->measuredWrench(); },
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
  logger.addLogEntry("perf_" + name_, [this]() { return runTime_; });

  logger.addLogEntry(name_ + "_error_dcm_average", [this]() { return dcmAverageError_; });
  logger.addLogEntry(name_ + "_error_dcm_pos", [this]() { return dcmError_; });
  logger.addLogEntry(name_ + "_error_dcm_vel", [this]() { return dcmVelError_; });
  logger.addLogEntry(name_ + "_error_dfz_force", [this]() { return dfzForceError_; });
  logger.addLogEntry(name_ + "_error_dfz_height", [this]() { return dfzHeightError_; });
  logger.addLogEntry(name_ + "_error_vdc", [this]() { return vdcHeightError_; });
  logger.addLogEntry(name_ + "_admittance_cop", [this]() { return c_.copAdmittance; });
  zmpcc_.addToLogger(logger, name_);
  logger.addLogEntry(name_ + "_admittance_dfz", [this]() { return c_.dfzAdmittance; });
  logger.addLogEntry(name_ + "_dcmDerivator_filtered", [this]() { return dcmDerivator_.eval(); });
  logger.addLogEntry(name_ + "_dcmDerivator_timeConstant", [this]() { return dcmDerivator_.timeConstant(); });
  logger.addLogEntry(name_ + "_dcmIntegrator_timeConstant", [this]() { return dcmIntegrator_.timeConstant(); });
  logger.addLogEntry(name_ + "_dcmTracking_derivGain", [this]() { return c_.dcmDerivGain; });
  logger.addLogEntry(name_ + "_dcmTracking_integralGain", [this]() { return c_.dcmIntegralGain; });
  logger.addLogEntry(name_ + "_dcmTracking_propGain", [this]() { return c_.dcmPropGain; });
  logger.addLogEntry(name_ + "_dfz_damping", [this]() { return c_.dfzDamping; });
  logger.addLogEntry(name_ + "_fdqp_weights_ankleTorque",
                     [this]() { return std::pow(c_.fdqpWeights.ankleTorqueSqrt, 2); });
  logger.addLogEntry(name_ + "_fdqp_weights_netWrench", [this]() { return std::pow(c_.fdqpWeights.netWrenchSqrt, 2); });
  logger.addLogEntry(name_ + "_fdqp_weights_pressure", [this]() { return std::pow(c_.fdqpWeights.pressureSqrt, 2); });
  logger.addLogEntry(name_ + "_vdc_frequency", [this]() { return c_.vdcFrequency; });
  logger.addLogEntry(name_ + "_vdc_stiffness", [this]() { return c_.vdcStiffness; });
  logger.addLogEntry(name_ + "_wrench", [this]() { return distribWrench_; });
  logger.addLogEntry(name_ + "_support_min", [this]() { return supportMin_; });
  logger.addLogEntry(name_ + "_support_max", [this]() { return supportMax_; });
  logger.addLogEntry(name_ + "_left_foot_ratio", [this]() { return leftFootRatio_; });

  // Stabilizer targets
  logger.addLogEntry(name_ + "_target_pendulum_com", [this]() { return comTarget_; });
  logger.addLogEntry(name_ + "_target_pendulum_comd", [this]() { return comdTarget_; });
  logger.addLogEntry(name_ + "_target_pendulum_comdd", [this]() { return comddTarget_; });
  logger.addLogEntry(name_ + "_target_pendulum_dcm", [this]() { return dcmTarget_; });
  logger.addLogEntry(name_ + "_target_pendulum_omega", [this]() { return omega_; });
  logger.addLogEntry(name_ + "_target_pendulum_zmp", [this]() { return zmpTarget_; });

  logger.addLogEntry(name_ + "_contactState", [this]() -> double {
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
  logger.addLogEntry(name_ + "_controlRobot_LeftFoot", [this]() { return robot().surfacePose("LeftFoot"); });
  logger.addLogEntry(name_ + "_controlRobot_LeftFootCenter",
                     [this]() { return robot().surfacePose("LeftFootCenter"); });
  logger.addLogEntry(name_ + "_controlRobot_RightFoot", [this]() { return robot().surfacePose("RightFoot"); });
  logger.addLogEntry(name_ + "_controlRobot_RightFootCenter",
                     [this]() { return robot().surfacePose("RightFootCenter"); });
  logger.addLogEntry(name_ + "_controlRobot_com", [this]() { return robot().com(); });
  logger.addLogEntry(name_ + "_controlRobot_comd", [this]() { return robot().comVelocity(); });
  logger.addLogEntry(name_ + "_controlRobot_posW", [this]() { return robot().posW(); });

  logger.addLogEntry(name_ + "_realRobot_LeftFoot", [this]() { return realRobot().surfacePose("LeftFoot"); });
  logger.addLogEntry(name_ + "_realRobot_LeftFootCenter",
                     [this]() { return realRobot().surfacePose("LeftFootCenter"); });
  logger.addLogEntry(name_ + "_realRobot_RightFoot", [this]() { return realRobot().surfacePose("RightFoot"); });
  logger.addLogEntry(name_ + "_realRobot_RightFootCenter",
                     [this]() { return realRobot().surfacePose("RightFootCenter"); });
  logger.addLogEntry(name_ + "_realRobot_com", [this]() { return measuredCoM_; });
  logger.addLogEntry(name_ + "_realRobot_comd", [this]() { return measuredCoMd_; });
  logger.addLogEntry(name_ + "_realRobot_dcm", [this]() -> Eigen::Vector3d { return measuredDCM_; });
  logger.addLogEntry(name_ + "_realRobot_posW", [this]() { return realRobot().posW(); });
  logger.addLogEntry(name_ + "_realRobot_wrench", [this]() { return measuredNetWrench_; });
  logger.addLogEntry(name_ + "_realRobot_zmp", [this]() { return measuredZMP_; });

  MetaTask::addToLogger(*comTask, logger);
  MetaTask::addToLogger(*pelvisTask, logger);
  MetaTask::addToLogger(*torsoTask, logger);
}

void StabilizerTask::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntry("perf_" + name_);
  logger.removeLogEntry(name_ + "_contactState");
  logger.removeLogEntry(name_ + "_error_dcm_average");
  logger.removeLogEntry(name_ + "_error_dcm_pos");
  logger.removeLogEntry(name_ + "_error_dcm_vel");
  logger.removeLogEntry(name_ + "_error_dfz_force");
  logger.removeLogEntry(name_ + "_error_dfz_height");
  logger.removeLogEntry(name_ + "_error_vdc");
  zmpcc_.removeFromLogger(logger, name_);
  logger.removeLogEntry(name_ + "_admittance_cop");
  logger.removeLogEntry(name_ + "_admittance_dfz");
  logger.removeLogEntry(name_ + "_dcmDerivator_filtered");
  logger.removeLogEntry(name_ + "_dcmDerivator_timeConstant");
  logger.removeLogEntry(name_ + "_dcmIntegrator_timeConstant");
  logger.removeLogEntry(name_ + "_dcmTracking_derivGain");
  logger.removeLogEntry(name_ + "_dcmTracking_integralGain");
  logger.removeLogEntry(name_ + "_dcmTracking_propGain");
  logger.removeLogEntry(name_ + "_dfz_damping");
  logger.removeLogEntry(name_ + "_fdqp_weights_ankleTorque");
  logger.removeLogEntry(name_ + "_fdqp_weights_netWrench");
  logger.removeLogEntry(name_ + "_fdqp_weights_pressure");
  logger.removeLogEntry(name_ + "_vdc_frequency");
  logger.removeLogEntry(name_ + "_vdc_stiffness");
  logger.removeLogEntry(name_ + "_wrench");
  logger.removeLogEntry(name_ + "_zmp");
  logger.removeLogEntry(name_ + "_support_min");
  logger.removeLogEntry(name_ + "_support_max");
  logger.removeLogEntry(name_ + "_left_foot_ratio");
  logger.removeLogEntry(name_ + "_target_pendulum_com");
  logger.removeLogEntry(name_ + "_target_pendulum_comd");
  logger.removeLogEntry(name_ + "_target_pendulum_comdd");
  logger.removeLogEntry(name_ + "_target_pendulum_dcm");
  logger.removeLogEntry(name_ + "_target_pendulum_omega");
  logger.removeLogEntry(name_ + "_target_pendulum_zmp");
  logger.removeLogEntry(name_ + "_controlRobot_LeftFoot");
  logger.removeLogEntry(name_ + "_controlRobot_LeftFootCenter");
  logger.removeLogEntry(name_ + "_controlRobot_RightFoot");
  logger.removeLogEntry(name_ + "_controlRobot_RightFootCenter");
  logger.removeLogEntry(name_ + "_controlRobot_com");
  logger.removeLogEntry(name_ + "_controlRobot_comd");
  logger.removeLogEntry(name_ + "_controlRobot_posW");
  logger.removeLogEntry(name_ + "_realRobot_LeftFoot");
  logger.removeLogEntry(name_ + "_realRobot_LeftFootCenter");
  logger.removeLogEntry(name_ + "_realRobot_RightFoot");
  logger.removeLogEntry(name_ + "_realRobot_RightFootCenter");
  logger.removeLogEntry(name_ + "_realRobot_com");
  logger.removeLogEntry(name_ + "_realRobot_comd");
  logger.removeLogEntry(name_ + "_realRobot_dcm");
  logger.removeLogEntry(name_ + "_realRobot_posW");
  logger.removeLogEntry(name_ + "_realRobot_wrench");
  logger.removeLogEntry(name_ + "_realRobot_zmp");

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
