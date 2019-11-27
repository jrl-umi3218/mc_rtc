/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is inspired by Stephane's Caron original implementation as part of
 * lipm_walking_controller <https://github.com/stephane-caron/lipm_walking_controller>
 */

#include <mc_rtc/gui.h>
#include <mc_rtc/gui/plot.h>
#include <mc_tasks/LIPMStabilizerTask.h>
#include <mc_tasks/MetaTaskLoader.h>

#include <chrono>

namespace mc_tasks
{
namespace lipm_stabilizer
{

using mc_signal::utils::clamp;
using mc_signal::utils::clampInPlace;

// Repeat static constexpr declarations
// Fixes https://github.com/stephane-caron/lipm_walking_controller/issues/21
// See also https://stackoverflow.com/q/8016780
constexpr double StabilizerTask::MAX_AVERAGE_DCM_ERROR;
constexpr double StabilizerTask::MAX_COM_ADMITTANCE;
constexpr double StabilizerTask::MAX_COP_ADMITTANCE;
constexpr double StabilizerTask::MAX_DCM_D_GAIN;
constexpr double StabilizerTask::MAX_DCM_I_GAIN;
constexpr double StabilizerTask::MAX_DCM_P_GAIN;
constexpr double StabilizerTask::MAX_DFZ_ADMITTANCE;
constexpr double StabilizerTask::MAX_DFZ_DAMPING;
constexpr double StabilizerTask::MAX_FDC_RX_VEL;
constexpr double StabilizerTask::MAX_FDC_RY_VEL;
constexpr double StabilizerTask::MAX_FDC_RZ_VEL;
constexpr double StabilizerTask::MAX_ZMPCC_COM_OFFSET;
constexpr double StabilizerTask::MIN_DS_PRESSURE;
constexpr double StabilizerTask::MIN_NET_TOTAL_FORCE_ZMP;
constexpr double StabilizerTask::GRAVITY;

namespace
{
inline Eigen::Vector2d vecFromError(const Eigen::Vector3d & error)
{
  double x = -std::round(error.x() * 1000.);
  double y = -std::round(error.y() * 1000.);
  return Eigen::Vector2d{x, y};
}

const Eigen::Vector3d e_z{0., 0., 1.};
} // namespace

StabilizerTask::StabilizerTask(const mc_rbdyn::Robots & robots,
                               const mc_rbdyn::Robots & realRobots,
                               unsigned int robotIndex,
                               const std::string & leftSurface,
                               const std::string & rightSurface,
                               double dt)
: robots_(robots), realRobots_(realRobots), robotIndex_(robotIndex), leftFootSurface_(leftSurface),
  rightFootSurface_(rightSurface), dcmIntegrator_(dt, /* timeConstant = */ 15.),
  dcmDerivator_(dt, /* timeConstant = */ 1.), dt_(dt), mass_(robots.robot(robotIndex).mass())
{
  type_ = "Stabilizer";
  name_ = "Stabilizer";

  vertical_ = gravity_ / gravity_.norm();

  comTask.reset(new mc_tasks::CoMTask(robots, robotIndex_));
  leftFootTask.reset(new mc_tasks::force::CoPTask(leftFootSurface_, robots, robotIndex_));
  rightFootTask.reset(new mc_tasks::force::CoPTask(rightFootSurface_, robots, robotIndex_));

  std::string pelvisBodyName = robot().mb().body(0).name();
  pelvisTask = std::make_shared<mc_tasks::OrientationTask>(pelvisBodyName, robots_, robotIndex_);
  std::string torsoName = "CHEST_LINK1";
  torsoTask = std::make_shared<mc_tasks::OrientationTask>(torsoName, robots_, robotIndex_);
}

StabilizerTask::~StabilizerTask() {}

void StabilizerTask::reset()
{
  comTask->reset();
  rightFootTask->reset();
  leftFootTask->reset();
  pelvisTask->reset();
  torsoTask->reset();

  // Add upper-body tasks
  double pelvisStiffness = 10;
  double pelvisWeight = 100;
  pelvisTask->stiffness(pelvisStiffness);
  pelvisTask->weight(pelvisWeight);

  double torsoStiffness = 10;
  double torsoWeight = 100;
  torsoTask->stiffness(torsoStiffness);
  torsoTask->weight(torsoWeight);

  comTask->selectActiveJoints(c_.comActiveJoints);
  comTask->setGains(c_.comStiffness, 2 * c_.comStiffness.cwiseSqrt());
  comTask->weight(c_.comWeight);

  leftFootTask->maxAngularVel({MAX_FDC_RX_VEL, MAX_FDC_RY_VEL, MAX_FDC_RZ_VEL});
  rightFootTask->maxAngularVel({MAX_FDC_RX_VEL, MAX_FDC_RY_VEL, MAX_FDC_RZ_VEL});

  dcmDerivator_.setZero();
  dcmIntegrator_.saturation(MAX_AVERAGE_DCM_ERROR);
  dcmIntegrator_.setZero();
  zmpccIntegrator_.saturation(MAX_ZMPCC_COM_OFFSET);
  zmpccIntegrator_.setZero();

  Eigen::Vector3d staticForce = -mass_ * gravity_;

  dcmAverageError_ = Eigen::Vector3d::Zero();
  dcmError_ = Eigen::Vector3d::Zero();
  dcmVelError_ = Eigen::Vector3d::Zero();
  dfzForceError_ = 0.;
  dfzHeightError_ = 0.;
  distribWrench_ = {comTarget_.cross(staticForce), staticForce};
  vdcHeightError_ = 0.;
  zmpError_ = Eigen::Vector3d::Zero();
  zmpccCoMAccel_ = Eigen::Vector3d::Zero();
  zmpccCoMOffset_ = Eigen::Vector3d::Zero();
  zmpccCoMVel_ = Eigen::Vector3d::Zero();
  zmpccError_ = Eigen::Vector3d::Zero();

  omega_ = std::sqrt(robot().mbc().gravity.z() / robot().com().z());

  wrenchFaceMatrix(c_.sole);
}

void StabilizerTask::dimWeight(const Eigen::VectorXd & /* dim */)
{
  LOG_ERROR_AND_THROW(std::runtime_error, "dimWeight not implemented for task " << type_);
}

Eigen::VectorXd StabilizerTask::dimWeight() const
{
  LOG_ERROR_AND_THROW(std::runtime_error, "dimWeight not implemented for task " << type_);
}

void StabilizerTask::selectActiveJoints(mc_solver::QPSolver & solver,
                                        const std::vector<std::string> & activeJointsName,
                                        const std::map<std::string, std::vector<std::array<int, 2>>> & activeDofs)
{
  LOG_ERROR_AND_THROW(std::runtime_error, "Task " << name_ << " does not implement selectActiveJoints");
}

void StabilizerTask::selectUnactiveJoints(mc_solver::QPSolver & solver,
                                          const std::vector<std::string> & unactiveJointsName,
                                          const std::map<std::string, std::vector<std::array<int, 2>>> & unactiveDofs)
{
  LOG_ERROR_AND_THROW(std::runtime_error, "Task " << name_ << " does not implement selectUnactiveJoints");
}

void StabilizerTask::resetJointsSelector(mc_solver::QPSolver & solver)
{
  comTask->resetJointsSelector(solver);
  leftFootTask->resetJointsSelector(solver);
  rightFootTask->resetJointsSelector(solver);
}

Eigen::VectorXd StabilizerTask::eval() const
{
  LOG_ERROR_AND_THROW(std::runtime_error, "eval not implemented for task " << type_);
}

Eigen::VectorXd StabilizerTask::speed() const
{
  LOG_ERROR_AND_THROW(std::runtime_error, "speed not implemented for task " << type_);
}

void StabilizerTask::resetConfiguration(const mc_rbdyn::lipm_stabilizer::StabilizerConfiguration & config)
{
  c_ = config;
}

void StabilizerTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  configure(config);
}

void StabilizerTask::addToSolver(mc_solver::QPSolver & solver)
{
  MetaTask::addToSolver(*comTask, solver);
  MetaTask::addToSolver(*leftFootTask, solver);
  MetaTask::addToSolver(*rightFootTask, solver);
  MetaTask::addToSolver(*pelvisTask, solver);
  MetaTask::addToSolver(*torsoTask, solver);
}

void StabilizerTask::removeFromSolver(mc_solver::QPSolver & solver)
{
  MetaTask::removeFromSolver(*comTask, solver);
  MetaTask::removeFromSolver(*leftFootTask, solver);
  MetaTask::removeFromSolver(*rightFootTask, solver);
  MetaTask::removeFromSolver(*pelvisTask, solver);
  MetaTask::removeFromSolver(*torsoTask, solver);
}

void StabilizerTask::update()
{
  updateState(realRobots_.robot().com(), realRobots_.robot().comVelocity(), leftFootRatio());

  // Run stabilizer
  run();

  MetaTask::update(*comTask);
  MetaTask::update(*leftFootTask);
  MetaTask::update(*rightFootTask);
  MetaTask::update(*pelvisTask);
  MetaTask::update(*torsoTask);

  t_ += dt_;
}

void StabilizerTask::addToLogger(mc_rtc::Logger & logger)
{
  logger.addLogEntry("stabilizer_contactState", [this]() -> double {
    switch(contactState_)
    {
      case ContactState::DoubleSupport:
        return 0;
      case ContactState::LeftFoot:
        return 1;
      case ContactState::RightFoot:
        return -1;
      default:
        return -3;
    }
  });
  logger.addLogEntry("error_dcm_average", [this]() { return dcmAverageError_; });
  logger.addLogEntry("error_dcm_pos", [this]() { return dcmError_; });
  logger.addLogEntry("error_dcm_vel", [this]() { return dcmVelError_; });
  logger.addLogEntry("error_dfz_force", [this]() { return dfzForceError_; });
  logger.addLogEntry("error_dfz_height", [this]() { return dfzHeightError_; });
  logger.addLogEntry("error_vdc", [this]() { return vdcHeightError_; });
  logger.addLogEntry("error_zmp", [this]() { return zmpError_; });
  logger.addLogEntry("perf_Stabilizer", [this]() { return runTime_; });
  logger.addLogEntry("stabilizer_admittance_com", [this]() { return c_.comAdmittance; });
  logger.addLogEntry("stabilizer_admittance_cop", [this]() { return c_.copAdmittance; });
  logger.addLogEntry("stabilizer_admittance_dfz", [this]() { return c_.dfzAdmittance; });
  logger.addLogEntry("stabilizer_dcmDerivator_filtered", [this]() { return dcmDerivator_.eval(); });
  logger.addLogEntry("stabilizer_dcmDerivator_raw", [this]() { return dcmDerivator_.raw(); });
  logger.addLogEntry("stabilizer_dcmDerivator_timeConstant", [this]() { return dcmDerivator_.timeConstant(); });
  logger.addLogEntry("stabilizer_dcmIntegrator_timeConstant", [this]() { return dcmIntegrator_.timeConstant(); });
  logger.addLogEntry("stabilizer_dcmTracking_derivGain", [this]() { return c_.dcmDerivGain; });
  logger.addLogEntry("stabilizer_dcmTracking_integralGain", [this]() { return c_.dcmIntegralGain; });
  logger.addLogEntry("stabilizer_dcmTracking_propGain", [this]() { return c_.dcmPropGain; });
  logger.addLogEntry("stabilizer_dfz_damping", [this]() { return c_.dfzDamping; });
  logger.addLogEntry("stabilizer_fdqp_weights_ankleTorque",
                     [this]() { return std::pow(c_.fdqpWeights.ankleTorqueSqrt, 2); });
  logger.addLogEntry("stabilizer_fdqp_weights_netWrench",
                     [this]() { return std::pow(c_.fdqpWeights.netWrenchSqrt, 2); });
  logger.addLogEntry("stabilizer_fdqp_weights_pressure", [this]() { return std::pow(c_.fdqpWeights.pressureSqrt, 2); });
  logger.addLogEntry("stabilizer_vdc_frequency", [this]() { return c_.vdcFrequency; });
  logger.addLogEntry("stabilizer_vdc_stiffness", [this]() { return c_.vdcStiffness; });
  logger.addLogEntry("stabilizer_wrench", [this]() { return distribWrench_; });
  logger.addLogEntry("stabilizer_zmp", [this]() { return zmp(); });
  logger.addLogEntry("stabilizer_zmpcc_comAccel", [this]() { return zmpccCoMAccel_; });
  logger.addLogEntry("stabilizer_zmpcc_comOffset", [this]() { return zmpccCoMOffset_; });
  logger.addLogEntry("stabilizer_zmpcc_comVel", [this]() { return zmpccCoMVel_; });
  logger.addLogEntry("stabilizer_zmpcc_error", [this]() { return zmpccError_; });
  logger.addLogEntry("stabilizer_zmpcc_leakRate", [this]() { return zmpccIntegrator_.rate(); });

  logger.addLogEntry("controlRobot_LeftFoot", [this]() { return robot().surfacePose("LeftFoot"); });
  logger.addLogEntry("controlRobot_LeftFootCenter", [this]() { return robot().surfacePose("LeftFootCenter"); });
  logger.addLogEntry("controlRobot_RightFoot", [this]() { return robot().surfacePose("RightFoot"); });
  logger.addLogEntry("controlRobot_RightFootCenter", [this]() { return robot().surfacePose("RightFootCenter"); });
  logger.addLogEntry("controlRobot_com", [this]() { return robot().com(); });
  logger.addLogEntry("controlRobot_comd", [this]() { return robot().comVelocity(); });
  logger.addLogEntry("controlRobot_posW", [this]() { return robot().posW(); });
  logger.addLogEntry("left_foot_ratio", [this]() { return leftFootRatio_; });
  logger.addLogEntry("pendulum_com", [this]() { return comTarget_; });
  logger.addLogEntry("pendulum_comd", [this]() { return comdTarget_; });
  logger.addLogEntry("pendulum_comdd", [this]() { return comddTarget_; });
  logger.addLogEntry("pendulum_dcm", [this]() { return dcmTarget_; });
  logger.addLogEntry("pendulum_omega", [this]() { return omega_; });
  logger.addLogEntry("pendulum_zmp", [this]() { return zmpTarget_; });
  logger.addLogEntry("realRobot_LeftFoot", [this]() { return realRobot().surfacePose("LeftFoot"); });
  logger.addLogEntry("realRobot_LeftFootCenter", [this]() { return realRobot().surfacePose("LeftFootCenter"); });
  logger.addLogEntry("realRobot_RightFoot", [this]() { return realRobot().surfacePose("RightFoot"); });
  logger.addLogEntry("realRobot_RightFootCenter", [this]() { return realRobot().surfacePose("RightFootCenter"); });
  logger.addLogEntry("realRobot_com", [this]() { return measuredCoM_; });
  logger.addLogEntry("realRobot_comd", [this]() { return measuredCoMd_; });
  logger.addLogEntry("realRobot_dcm", [this]() -> Eigen::Vector3d { return measuredDCM_; });
  logger.addLogEntry("realRobot_posW", [this]() { return realRobot().posW(); });
  logger.addLogEntry("realRobot_wrench", [this]() { return measuredNetWrench_; });
  logger.addLogEntry("realRobot_zmp", [this]() { return measuredZMP_; });

  comTask->addToLogger(logger);
  leftFootTask->addToLogger(logger);
  rightFootTask->addToLogger(logger);
}

void StabilizerTask::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntry("stabilizer_contactState");
  logger.removeLogEntry("error_dcm_average");
  logger.removeLogEntry("error_dcm_pos");
  logger.removeLogEntry("error_dcm_vel");
  logger.removeLogEntry("error_dfz_force");
  logger.removeLogEntry("error_dfz_height");
  logger.removeLogEntry("error_vdc");
  logger.removeLogEntry("error_zmp");
  logger.removeLogEntry("perf_Stabilizer");
  logger.removeLogEntry("stabilizer_admittance_com");
  logger.removeLogEntry("stabilizer_admittance_cop");
  logger.removeLogEntry("stabilizer_admittance_dfz");
  logger.removeLogEntry("stabilizer_dcmDerivator_filtered");
  logger.removeLogEntry("stabilizer_dcmDerivator_raw");
  logger.removeLogEntry("stabilizer_dcmDerivator_timeConstant");
  logger.removeLogEntry("stabilizer_dcmIntegrator_timeConstant");
  logger.removeLogEntry("stabilizer_dcmTracking_derivGain");
  logger.removeLogEntry("stabilizer_dcmTracking_integralGain");
  logger.removeLogEntry("stabilizer_dcmTracking_propGain");
  logger.removeLogEntry("stabilizer_dfz_damping");
  logger.removeLogEntry("stabilizer_fdqp_weights_ankleTorque");
  logger.removeLogEntry("stabilizer_fdqp_weights_netWrench");
  logger.removeLogEntry("stabilizer_fdqp_weights_pressure");
  logger.removeLogEntry("stabilizer_vdc_frequency");
  logger.removeLogEntry("stabilizer_vdc_stiffness");
  logger.removeLogEntry("stabilizer_wrench");
  logger.removeLogEntry("stabilizer_zmp");
  logger.removeLogEntry("stabilizer_zmpcc_comAccel");
  logger.removeLogEntry("stabilizer_zmpcc_comOffset");
  logger.removeLogEntry("stabilizer_zmpcc_comVel");
  logger.removeLogEntry("stabilizer_zmpcc_error");
  logger.removeLogEntry("stabilizer_zmpcc_leakRate");
}

void StabilizerTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  using namespace mc_rtc::gui;

  gui.addElement({"Stabilizer", "Main"}, Button("Disable stabilizer", [this]() { disable(); }),
                 Button("Reconfigure", [this]() { reconfigure(); }),
                 Button("Reset DCM integrator", [this]() { dcmIntegrator_.setZero(); }),
                 ArrayInput("Foot admittance", {"CoPx", "CoPy"},
                            [this]() -> Eigen::Vector2d {
                              return {c_.copAdmittance.x(), c_.copAdmittance.y()};
                            },
                            [this](const Eigen::Vector2d & a) {
                              c_.copAdmittance.x() = clamp(a(0), 0., MAX_COP_ADMITTANCE);
                              c_.copAdmittance.y() = clamp(a(1), 0., MAX_COP_ADMITTANCE);
                            }),
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
  gui.addElement({"Stabilizer", "Advanced"}, Button("Disable stabilizer", [this]() { disable(); }),
                 Button("Reconfigure", [this]() { reconfigure(); }),
                 Button("Reset CoM integrator", [this]() { zmpccIntegrator_.setZero(); }),
                 ArrayInput("CoM admittance", {"Ax", "Ay"}, [this]() { return c_.comAdmittance; },
                            [this](const Eigen::Vector2d & a) {
                              c_.comAdmittance.x() = clamp(a.x(), 0., MAX_COM_ADMITTANCE);
                              c_.comAdmittance.y() = clamp(a.y(), 0., MAX_COM_ADMITTANCE);
                            }),
                 Checkbox("Apply CoM admittance only in double support?", [this]() { return zmpccOnlyDS_; },
                          [this]() { zmpccOnlyDS_ = !zmpccOnlyDS_; }),
                 NumberInput("CoM integrator leak rate [Hz]", [this]() { return zmpccIntegrator_.rate(); },
                             [this](double T) { zmpccIntegrator_.rate(T); }),
                 ArrayInput("DCM pole placement", {"Pole1", "Pole2", "Pole3", "Lag [Hz]"},
                            [this]() -> Eigen::VectorXd { return polePlacement_; },
                            [this](const Eigen::VectorXd & polePlacement) {
                              double alpha = clamp(polePlacement(0), -20., -0.1);
                              double beta = clamp(polePlacement(1), -20., -0.1);
                              double gamma = clamp(polePlacement(2), -20., -0.1);
                              double lagFreq = clamp(polePlacement(3), 1., 200.);
                              polePlacement_ = {alpha, beta, gamma, lagFreq};

                              double denom = omega_ * lagFreq;
                              double T_integ = dcmIntegrator_.timeConstant();

                              // Gains K_z for the ZMP feedback (Delta ZMP = K_z * Delta DCM)
                              double zmpPropGain =
                                  (alpha * beta + beta * gamma + gamma * alpha + omega_ * lagFreq) / denom;
                              double zmpIntegralGain = -(alpha * beta * gamma) / denom;
                              double zmpDerivGain = -(alpha + beta + gamma + lagFreq - omega_) / denom;

                              // Our gains K are for closed-loop DCM (Delta dot(DCM) = -K * Delta DCM)
                              c_.dcmPropGain = omega_ * (zmpPropGain - 1.);
                              c_.dcmIntegralGain = omega_ * T_integ * zmpIntegralGain; // our integrator is an EMA
                              c_.dcmDerivGain = omega_ * zmpDerivGain;
                            }),
                 ArrayInput("Vertical drift compensation", {"frequency", "stiffness"},
                            [this]() -> Eigen::Vector2d {
                              return {c_.vdcFrequency, c_.vdcStiffness};
                            },
                            [this](const Eigen::Vector2d & v) {
                              c_.vdcFrequency = clamp(v(0), 0., 10.);
                              c_.vdcStiffness = clamp(v(1), 0., 1e4);
                            }));

  gui.addElement(
      {"Stabilizer", "Debug"}, Button("Disable stabilizer", [this]() { disable(); }),
      Button("Reconfigure", [this]() { reconfigure(); }),
      Button("Add Plot DCM-ZMP Tracking (x)",
             [this, &gui]() {
               gui.addPlot("DCM-ZMP Tracking (x)", mc_rtc::gui::plot::X("t", [this]() { return t_; }),
                           mc_rtc::gui::plot::Y("dcm_ref", [this]() { return dcmTarget_.x(); }, Color::Red),
                           mc_rtc::gui::plot::Y("dcm_mes", [this]() { return measuredDCM_.x(); }, Color::Magenta),
                           mc_rtc::gui::plot::Y("zmp_ref", [this]() { return zmpTarget_.x(); }, Color::Blue),
                           mc_rtc::gui::plot::Y("zmp_mes", [this]() { return measuredZMP_.x(); }, Color::Cyan));
             }),
      Button("Remove Plot DCM-ZMP Tracking (x)", [&gui]() { gui.removePlot("DCM-ZMP Tracking (x)"); }),
      Button("Plot DCM-ZMP Tracking (y)",
             [this, &gui]() {
               gui.addPlot("DCM-ZMP Tracking (y)", mc_rtc::gui::plot::X("t", [this]() { return t_; }),
                           mc_rtc::gui::plot::Y("dcm_ref", [this]() { return dcmTarget_.y(); }, Color::Red),
                           mc_rtc::gui::plot::Y("dcm_mes", [this]() { return measuredDCM_.y(); }, Color::Magenta),
                           mc_rtc::gui::plot::Y("zmp_ref", [this]() { return zmpTarget_.y(); }, Color::Blue),
                           mc_rtc::gui::plot::Y("zmp_mes", [this]() { return measuredZMP_.y(); }, Color::Cyan));
             }),
      Button("Remove Plot DCM-ZMP Tracking (y)", [&gui]() { gui.removePlot("DCM-ZMP Tracking (y)"); }),
      ArrayLabel("CoM offset [mm]", {"x", "y"}, [this]() { return vecFromError(zmpccCoMOffset_); }),
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

  gui.addElement({"Stabilizer", "Markers", "CoM-DCM"},
                 Arrow("Pendulum_CoM", pendulumArrowConfig, [this]() -> Eigen::Vector3d { return zmpTarget_; },
                       [this]() -> Eigen::Vector3d { return comTarget_; }),
                 Point3D("Measured_CoM", PointConfig(Color::Green, COM_POINT_SIZE), [this]() { return measuredCoM_; }),
                 Point3D("Pendulum_DCM", PointConfig(Color::Yellow, DCM_POINT_SIZE), [this]() { return dcmTarget_; }),
                 Point3D("Measured_DCM", PointConfig(Color::Green, DCM_POINT_SIZE),
                         [this]() -> Eigen::Vector3d { return measuredCoM_ + measuredCoMd_ / omega_; }));

  gui.addElement(
      {"Stabilizer", "Markers", "Net wrench"},
      Point3D("Stabilizer_ZMP", PointConfig(Color::Magenta, 0.02), [this]() { return this->zmp(); }),
      Point3D("Measured_ZMP", PointConfig(Color::Red, 0.02), [this]() -> Eigen::Vector3d { return measuredZMP_; }),
      Arrow("Measured_ZMPForce", netWrenchForceArrowConfig, [this]() -> Eigen::Vector3d { return measuredZMP_; },
            [this, FORCE_SCALE]() -> Eigen::Vector3d {
              return measuredZMP_ + FORCE_SCALE * measuredNetWrench_.force();
            }));

  gui.addElement(
      {"Stabilizer", "Markers", "Foot wrenches"},
      Point3D("Stabilizer_LeftCoP", PointConfig(Color::Magenta, 0.01), [this]() { return leftFootTask->targetCoPW(); }),
      Force("Measured_LeftCoPForce", copForceConfig, [this]() { return this->robot().surfaceWrench("LeftFootCenter"); },
            [this]() { return sva::PTransformd(this->robot().copW("LeftFootCenter")); }),
      Point3D("Stabilizer_RightCoP", PointConfig(Color::Magenta, 0.01),
              [this]() { return rightFootTask->targetCoPW(); }),
      Force("Measured_RightCoPForce", copForceConfig,
            [this]() { return this->robot().surfaceWrench("RightFootCenter"); },
            [this]() { return sva::PTransformd(this->robot().copW("RightFootCenter")); }));

  gui.addElement({"Stabilizer", "Markers", "Contacts"},
                 Polygon("SupportContacts", mc_rtc::gui::Color(0., 1., 0.), [this]() { return supportPolygons_; }));
}

void StabilizerTask::disable()
{
  c_.comAdmittance.setZero();
  c_.copAdmittance.setZero();
  c_.dcmDerivGain = 0.;
  c_.dcmIntegralGain = 0.;
  c_.dcmPropGain = 0.;
  c_.dfzAdmittance = 0.;
  c_.vdcFrequency = 0.;
  c_.vdcStiffness = 0.;
}

void StabilizerTask::configure(const mc_rtc::Configuration & config)
{
  config_ = config;
  reconfigure();
}

void StabilizerTask::reconfigure()
{
  // Load stabilizer configuration
  c_.load(config_);

  dcmDerivator_.timeConstant(c_.dcmDerivatorTimeConstant);
  dcmIntegrator_.timeConstant(c_.dcmIntegratorTimeConstant);

  // Configure contacts
  double hw = c_.sole.halfWidth;
  double hl = c_.sole.halfLength;
  leftFootContact_.surfaceName = leftFootTask->surface();
  leftFootContact_.halfLength = hl;
  leftFootContact_.halfWidth = hw;
  rightFootContact_.surfaceName = rightFootTask->surface();
  rightFootContact_.halfLength = hl;
  rightFootContact_.halfWidth = hw;
}

void StabilizerTask::checkGains()
{
  clampInPlace(c_.comAdmittance.x(), 0., MAX_COM_ADMITTANCE, "CoM x-admittance");
  clampInPlace(c_.comAdmittance.y(), 0., MAX_COM_ADMITTANCE, "CoM y-admittance");
  clampInPlace(c_.copAdmittance.x(), 0., MAX_COP_ADMITTANCE, "CoP x-admittance");
  clampInPlace(c_.copAdmittance.y(), 0., MAX_COP_ADMITTANCE, "CoP y-admittance");
  clampInPlace(c_.dcmDerivGain, 0., MAX_DCM_D_GAIN, "DCM deriv x-gain");
  clampInPlace(c_.dcmIntegralGain, 0., MAX_DCM_I_GAIN, "DCM integral x-gain");
  clampInPlace(c_.dcmPropGain, 0., MAX_DCM_P_GAIN, "DCM prop x-gain");
  clampInPlace(c_.dfzAdmittance, 0., MAX_DFZ_ADMITTANCE, "DFz admittance");
}

void StabilizerTask::setContacts(ContactState state)
{
  contactState_ = state;

  auto footStepPolygon = [](const Contact & contact) {
    std::vector<Eigen::Vector3d> polygon;
    polygon.push_back(contact.vertex0());
    polygon.push_back(contact.vertex1());
    polygon.push_back(contact.vertex2());
    polygon.push_back(contact.vertex3());
    return polygon;
  };

  auto configureFootSupport = [this](std::shared_ptr<mc_tasks::force::CoPTask> footTask, Contact & contact) {
    footTask->reset();
    footTask->admittance(contactAdmittance());
    footTask->setGains(c_.contactStiffness, c_.contactDamping);
    footTask->weight(c_.contactWeight);
    // set contact pose as well
    contact.pose = footTask->surfacePose();
  };

  auto configureSwingFoot = [this](std::shared_ptr<mc_tasks::force::CoPTask> footTask) {
    footTask->reset();
    footTask->stiffness(c_.swingFootStiffness); // sets damping as well
    footTask->weight(c_.swingFootWeight);
  };

  supportPolygons_.clear();
  if(state == ContactState::DoubleSupport)
  {
    configureFootSupport(leftFootTask, leftFootContact_);
    configureFootSupport(rightFootTask, rightFootContact_);
    supportPolygons_.push_back(footStepPolygon(leftFootContact_));
    supportPolygons_.push_back(footStepPolygon(rightFootContact_));
  }
  else if(state == ContactState::LeftFoot)
  {
    configureFootSupport(leftFootTask, leftFootContact_);
    supportPolygons_.push_back(footStepPolygon(leftFootContact_));
    configureSwingFoot(rightFootTask);
  }
  else
  {
    configureFootSupport(rightFootTask, rightFootContact_);
    supportPolygons_.push_back(footStepPolygon(leftFootContact_));
    configureSwingFoot(leftFootTask);
  }
}

bool StabilizerTask::detectTouchdown(const std::shared_ptr<mc_tasks::force::CoPTask> footTask, const Contact & contact)
{
  const sva::PTransformd X_0_s = footTask->surfacePose();
  const sva::PTransformd & X_0_c = contact.pose;
  sva::PTransformd X_c_s = X_0_s * X_0_c.inv();
  double xDist = std::abs(X_c_s.translation().x());
  double yDist = std::abs(X_c_s.translation().y());
  double zDist = std::abs(X_c_s.translation().z());
  double pressure = footTask->measuredWrench().force().z();
  return (xDist < 0.03 && yDist < 0.03 && zDist < 0.03 && pressure > 50.);
}

void StabilizerTask::seekTouchdown(std::shared_ptr<mc_tasks::force::CoPTask> footTask)
{
  constexpr double MAX_VEL = 0.01; // [m] / [s]
  constexpr double TOUCHDOWN_PRESSURE = 50.; // [N]
  constexpr double DESIRED_AFZ = MAX_VEL / TOUCHDOWN_PRESSURE;
  if(footTask->measuredWrench().force().z() < TOUCHDOWN_PRESSURE)
  {
    auto a = footTask->admittance();
    double AFz = clamp(DESIRED_AFZ, 0., 1e-2, "Contact seeking admittance");
    footTask->admittance({a.couple(), {a.force().x(), a.force().y(), AFz}});
    footTask->targetForce({0., 0., TOUCHDOWN_PRESSURE});
  }
}

void StabilizerTask::setSupportFootGains()
{
  sva::MotionVecd vdcContactStiffness = {c_.contactStiffness.angular(),
                                         {c_.vdcStiffness, c_.vdcStiffness, c_.vdcStiffness}};
  switch(contactState_)
  {
    case ContactState::DoubleSupport:
      leftFootTask->admittance(contactAdmittance());
      leftFootTask->setGains(c_.contactStiffness, c_.contactDamping);
      rightFootTask->admittance(contactAdmittance());
      rightFootTask->setGains(c_.contactStiffness, c_.contactDamping);
      break;
    case ContactState::LeftFoot:
      leftFootTask->admittance(contactAdmittance());
      leftFootTask->setGains(vdcContactStiffness, c_.contactDamping);
      break;
    case ContactState::RightFoot:
      rightFootTask->admittance(contactAdmittance());
      rightFootTask->setGains(vdcContactStiffness, c_.contactDamping);
      break;
  }
}

void StabilizerTask::checkInTheAir()
{
  double LFz = leftFootTask->measuredWrench().force().z();
  double RFz = rightFootTask->measuredWrench().force().z();
  inTheAir_ = (LFz < MIN_DS_PRESSURE && RFz < MIN_DS_PRESSURE);
}

void StabilizerTask::updateZMPFrame()
{
  const sva::PTransformd & X_0_lc = leftFootContact_.pose;
  const sva::PTransformd & X_0_rc = rightFootContact_.pose;
  switch(contactState_)
  {
    case ContactState::DoubleSupport:
      zmpFrame_ = sva::interpolate(X_0_lc, X_0_rc, 0.5);
      break;
    case ContactState::LeftFoot:
      zmpFrame_ = X_0_lc;
      break;
    case ContactState::RightFoot:
      zmpFrame_ = X_0_rc;
      break;
  }
  measuredNetWrench_ = robots_.robot(robotIndex_).netWrench(sensorNames_);
  measuredZMP_ = computeZMP(measuredNetWrench_); // computeZMP
}

Eigen::Vector3d StabilizerTask::computeZMP(const sva::ForceVecd & wrench) const
{
  return robots_.robot(robotIndex_).zmp(wrench, zmpFrame_, MIN_NET_TOTAL_FORCE_ZMP);
}

void StabilizerTask::staticTarget(const Eigen::Vector3d & com)
{
  comTarget_ = com;
  comdTarget_ = Eigen::Vector3d::Zero();
  comddTarget_ = Eigen::Vector3d::Zero();
  // XXX should compute height, omega
  zmpTarget_ = Eigen::Vector3d{com.x(), com.y(), 0.};
  dcmTarget_ = comTarget_;
}

void StabilizerTask::run()
{
  using namespace std::chrono;
  auto startTime = high_resolution_clock::now();

  checkGains();
  checkInTheAir();
  setSupportFootGains();
  updateZMPFrame();
  auto desiredWrench = computeDesiredWrench();

  switch(contactState_)
  {
    case ContactState::DoubleSupport:
      distributeWrench(desiredWrench);
      break;
    case ContactState::LeftFoot:
      saturateWrench(desiredWrench, leftFootTask);
      rightFootTask->setZeroTargetWrench();
      break;
    case ContactState::RightFoot:
      saturateWrench(desiredWrench, rightFootTask);
      leftFootTask->setZeroTargetWrench();
      break;
  }

  updateCoMTaskZMPCC();
  updateFootForceDifferenceControl();

  auto endTime = high_resolution_clock::now();
  runTime_ = 1000. * duration_cast<duration<double>>(endTime - startTime).count();
}

void StabilizerTask::updateState(const Eigen::Vector3d & com, const Eigen::Vector3d & comd, double leftFootRatio)
{
  leftFootRatio_ = leftFootRatio;
  measuredCoM_ = com;
  measuredCoMd_ = comd;
  measuredDCM_ = measuredCoM_ + measuredCoMd_ / omega_;
}

sva::ForceVecd StabilizerTask::computeDesiredWrench()
{
  Eigen::Vector3d comError = comTarget_ - measuredCoM_;
  Eigen::Vector3d comdError = comdTarget_ - measuredCoMd_;
  dcmError_ = comError + comdError / omega_;
  dcmError_.z() = 0.;

  if(inTheAir_)
  {
    dcmDerivator_.setZero();
    dcmIntegrator_.append(Eigen::Vector3d::Zero());
  }
  else
  {
    zmpError_ = zmpTarget_ - measuredZMP_; // XXX: both in same plane?
    zmpError_.z() = 0.;
    dcmDerivator_.update(omega_ * (dcmError_ - zmpError_));
    dcmIntegrator_.append(dcmError_);
  }
  dcmAverageError_ = dcmIntegrator_.eval();
  dcmVelError_ = dcmDerivator_.eval();

  Eigen::Vector3d desiredCoMAccel = comddTarget_;
  desiredCoMAccel += omega_ * (c_.dcmPropGain * dcmError_ + comdError);
  desiredCoMAccel += omega_ * c_.dcmIntegralGain * dcmAverageError_;
  desiredCoMAccel += omega_ * c_.dcmDerivGain * dcmVelError_;
  auto desiredForce = mass_ * (desiredCoMAccel - gravity_);
  return {measuredCoM_.cross(desiredForce), desiredForce};
}

void StabilizerTask::distributeWrench(const sva::ForceVecd & desiredWrench)
{
  // Variables
  // ---------
  // x = [w_l_0 w_r_0] where
  // w_l_0: spatial force vector of left foot contact in inertial frame
  // w_r_0: spatial force vector of right foot contact in inertial frame
  //
  // Objective
  // ---------
  // Weighted minimization of the following tasks:
  // w_l_0 + w_r_0 == desiredWrench  -- realize desired contact wrench
  // w_l_lankle == 0 -- minimize left foot ankle torque (anisotropic weight)
  // w_r_rankle == 0 -- minimize right foot ankle torque (anisotropic weight)
  // (1 - lfr) * w_l_lc.z() == lfr * w_r_rc.z()
  //
  // Constraints
  // -----------
  // CWC X_0_lc* w_l_0 <= 0  -- left foot wrench within contact wrench cone
  // CWC X_0_rc* w_r_0 <= 0  -- right foot wrench within contact wrench cone
  // (X_0_lc* w_l_0).z() > minPressure  -- minimum left foot contact pressure
  // (X_0_rc* w_r_0).z() > minPressure  -- minimum right foot contact pressure

  const sva::PTransformd & X_0_lc = leftFootContact_.pose;
  const sva::PTransformd & X_0_rc = rightFootContact_.pose;
  sva::PTransformd X_0_lankle = leftFootContact_.anklePose();
  sva::PTransformd X_0_rankle = rightFootContact_.anklePose();

  constexpr unsigned NB_VAR = 6 + 6;
  constexpr unsigned COST_DIM = 6 + NB_VAR + 1;
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  A.setZero(COST_DIM, NB_VAR);
  b.setZero(COST_DIM);

  // |w_l_0 + w_r_0 - desiredWrench|^2
  auto A_net = A.block<6, 12>(0, 0);
  auto b_net = b.segment<6>(0);
  A_net.block<6, 6>(0, 0) = Eigen::Matrix6d::Identity();
  A_net.block<6, 6>(0, 6) = Eigen::Matrix6d::Identity();
  b_net = desiredWrench.vector();

  // |ankle torques|^2
  auto A_lankle = A.block<6, 6>(6, 0);
  auto A_rankle = A.block<6, 6>(12, 6);
  // anisotropic weights:  taux, tauy, tauz,   fx,   fy,   fz;
  A_lankle.diagonal() << 1., 1., 1e-4, 1e-3, 1e-3, 1e-4;
  A_rankle.diagonal() << 1., 1., 1e-4, 1e-3, 1e-3, 1e-4;
  A_lankle *= X_0_lankle.dualMatrix();
  A_rankle *= X_0_rankle.dualMatrix();

  // |(1 - lfr) * w_l_lc.force().z() - lfr * w_r_rc.force().z()|^2
  double lfr = leftFootRatio_;
  auto A_pressure = A.block<1, 12>(18, 0);
  A_pressure.block<1, 6>(0, 0) = (1 - lfr) * X_0_lc.dualMatrix().bottomRows<1>();
  A_pressure.block<1, 6>(0, 6) = -lfr * X_0_rc.dualMatrix().bottomRows<1>();

  // Apply weights
  A_net *= c_.fdqpWeights.netWrenchSqrt;
  b_net *= c_.fdqpWeights.netWrenchSqrt;
  A_lankle *= c_.fdqpWeights.ankleTorqueSqrt;
  A_rankle *= c_.fdqpWeights.ankleTorqueSqrt;
  // b_lankle = 0
  // b_rankle = 0
  A_pressure *= c_.fdqpWeights.pressureSqrt;
  // b_pressure = 0

  Eigen::MatrixXd Q = A.transpose() * A;
  Eigen::VectorXd c = -A.transpose() * b;

  constexpr unsigned NB_CONS = 16 + 16 + 2;
  Eigen::Matrix<double, NB_CONS, NB_VAR> A_ineq;
  Eigen::VectorXd b_ineq;
  A_ineq.setZero(NB_CONS, NB_VAR);
  b_ineq.setZero(NB_CONS);
  // CWC * w_l_lc <= 0
  A_ineq.block<16, 6>(0, 0) = wrenchFaceMatrix_ * X_0_lc.dualMatrix();
  // b_ineq.segment<16>(0) is already zero
  // CWC * w_r_rc <= 0
  A_ineq.block<16, 6>(16, 6) = wrenchFaceMatrix_ * X_0_rc.dualMatrix();
  // b_ineq.segment<16>(16) is already zero
  // w_l_lc.force().z() >= MIN_DS_PRESSURE
  A_ineq.block<1, 6>(32, 0) = -X_0_lc.dualMatrix().bottomRows<1>();
  b_ineq(32) = -MIN_DS_PRESSURE;
  // w_r_rc.force().z() >= MIN_DS_PRESSURE
  A_ineq.block<1, 6>(33, 6) = -X_0_rc.dualMatrix().bottomRows<1>();
  b_ineq(33) = -MIN_DS_PRESSURE;

  qpSolver_.problem(NB_VAR, 0, NB_CONS);
  Eigen::MatrixXd A_eq(0, 0);
  Eigen::VectorXd b_eq;
  b_eq.resize(0);

  bool solutionFound = qpSolver_.solve(Q, c, A_eq, b_eq, A_ineq, b_ineq, /* isDecomp = */ false);
  if(!solutionFound)
  {
    LOG_ERROR("[StabilizerTask] DS force distribution QP: solver found no solution");
    return;
  }

  Eigen::VectorXd x = qpSolver_.result();
  sva::ForceVecd w_l_0(x.segment<3>(0), x.segment<3>(3));
  sva::ForceVecd w_r_0(x.segment<3>(6), x.segment<3>(9));
  distribWrench_ = w_l_0 + w_r_0;

  sva::ForceVecd w_l_lc = X_0_lc.dualMul(w_l_0);
  sva::ForceVecd w_r_rc = X_0_rc.dualMul(w_r_0);
  Eigen::Vector2d leftCoP = (e_z.cross(w_l_lc.couple()) / w_l_lc.force()(2)).head<2>();
  Eigen::Vector2d rightCoP = (e_z.cross(w_r_rc.couple()) / w_r_rc.force()(2)).head<2>();
  leftFootTask->targetCoP(leftCoP);
  leftFootTask->targetForce(w_l_lc.force());
  rightFootTask->targetCoP(rightCoP);
  rightFootTask->targetForce(w_r_rc.force());
}

void StabilizerTask::saturateWrench(const sva::ForceVecd & desiredWrench,
                                    std::shared_ptr<mc_tasks::force::CoPTask> & footTask)
{
  constexpr unsigned NB_CONS = 16;
  constexpr unsigned NB_VAR = 6;

  // Variables
  // ---------
  // x = [w_0] where
  // w_0: spatial force vector of foot contact in inertial frame
  //
  // Objective
  // ---------
  // weighted minimization of |w_c - X_0_c* desiredWrench|^2
  //
  // Constraints
  // -----------
  // F X_0_c* w_0 <= 0    -- contact stability

  const sva::PTransformd & X_0_c = footTask->targetPose();

  Eigen::Matrix6d Q = Eigen::Matrix6d::Identity();
  Eigen::Vector6d c = -desiredWrench.vector();

  Eigen::MatrixXd A_ineq = wrenchFaceMatrix_ * X_0_c.dualMatrix();
  Eigen::VectorXd b_ineq;
  b_ineq.setZero(NB_CONS);

  qpSolver_.problem(NB_VAR, 0, NB_CONS);
  Eigen::MatrixXd A_eq(0, 0);
  Eigen::VectorXd b_eq;
  b_eq.resize(0);

  bool solutionFound = qpSolver_.solve(Q, c, A_eq, b_eq, A_ineq, b_ineq, /* isDecomp = */ true);
  if(!solutionFound)
  {
    LOG_ERROR("[StabilizerTask] SS force distribution QP: solver found no solution");
    return;
  }

  Eigen::VectorXd x = qpSolver_.result();
  sva::ForceVecd w_0(x.head<3>(), x.tail<3>());
  sva::ForceVecd w_c = X_0_c.dualMul(w_0);
  Eigen::Vector2d cop = (e_z.cross(w_c.couple()) / w_c.force()(2)).head<2>();
  footTask->targetCoP(cop);
  footTask->targetForce(w_c.force());
  distribWrench_ = w_0;
}

void StabilizerTask::updateCoMTaskZMPCC()
{
  if(zmpccOnlyDS_ && contactState_ != ContactState::DoubleSupport)
  {
    zmpccCoMAccel_.setZero();
    zmpccCoMVel_.setZero();
    zmpccIntegrator_.add(Eigen::Vector3d::Zero(), dt_); // leak to zero
  }
  else
  {
    auto distribZMP = computeZMP(distribWrench_);
    zmpccError_ = distribZMP - measuredZMP_;
    const Eigen::Matrix3d & R_0_c = zmpFrame_.rotation();
    const Eigen::Transpose<const Eigen::Matrix3d> R_c_0 = R_0_c.transpose();
    Eigen::Vector3d comAdmittance = {c_.comAdmittance.x(), c_.comAdmittance.y(), 0.};
    Eigen::Vector3d newVel = -R_c_0 * comAdmittance.cwiseProduct(R_0_c * zmpccError_);
    Eigen::Vector3d newAccel = (newVel - zmpccCoMVel_) / dt_;
    zmpccIntegrator_.add(newVel, dt_);
    zmpccCoMAccel_ = newAccel;
    zmpccCoMVel_ = newVel;
  }
  zmpccCoMOffset_ = zmpccIntegrator_.eval();
  comTask->com(comTarget_ + zmpccCoMOffset_);
  comTask->refVel(comdTarget_ + zmpccCoMVel_);
  comTask->refAccel(comddTarget_ + zmpccCoMAccel_);
}

void StabilizerTask::updateFootForceDifferenceControl()
{
  if(contactState_ != ContactState::DoubleSupport || inTheAir_)
  {
    dfzForceError_ = 0.;
    dfzHeightError_ = 0.;
    vdcHeightError_ = 0.;
    leftFootTask->refVelB({{0., 0., 0.}, {0., 0., 0.}});
    rightFootTask->refVelB({{0., 0., 0.}, {0., 0., 0.}});
    return;
  }

  double LFz_d = leftFootTask->targetWrench().force().z();
  double RFz_d = rightFootTask->targetWrench().force().z();
  double LFz = leftFootTask->measuredWrench().force().z();
  double RFz = rightFootTask->measuredWrench().force().z();
  dfzForceError_ = (LFz_d - RFz_d) - (LFz - RFz);

  double LTz_d = leftFootTask->targetPose().translation().z();
  double RTz_d = rightFootTask->targetPose().translation().z();
  double LTz = leftFootTask->surfacePose().translation().z();
  double RTz = rightFootTask->surfacePose().translation().z();
  dfzHeightError_ = (LTz_d - RTz_d) - (LTz - RTz);
  vdcHeightError_ = (LTz_d + RTz_d) - (LTz + RTz);

  double dz_ctrl = c_.dfzAdmittance * dfzForceError_ - c_.dfzDamping * dfzHeightError_;
  double dz_vdc = c_.vdcFrequency * vdcHeightError_;
  sva::MotionVecd velF = {{0., 0., 0.}, {0., 0., dz_ctrl}};
  sva::MotionVecd velT = {{0., 0., 0.}, {0., 0., dz_vdc}};
  leftFootTask->refVelB(0.5 * (velT - velF));
  rightFootTask->refVelB(0.5 * (velT + velF));
}

} // namespace lipm_stabilizer
} // namespace mc_tasks

namespace
{

static bool registered = mc_tasks::MetaTaskLoader::register_load_function(
    "lipm_stabilizer",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      unsigned robotIndex = config("robotIndex");
      auto & robot = solver.robots().robot(robotIndex);

      if(!config.has(robot.name()))
      {
        LOG_ERROR_AND_THROW(std::runtime_error,
                            "[StabilizerTask] Configuration does not exist for robot " << robot.name());
      }

      if(!config.has("surfaces"))
      {
        LOG_ERROR_AND_THROW(std::runtime_error, "[StabilizerTask] Missing \"surfaces\" configuration entry");
      }
      std::string left = config("surfaces")("left");
      std::string right = config("surfaces")("right");

      auto t = std::make_shared<mc_tasks::lipm_stabilizer::StabilizerTask>(solver.robots(), solver.realRobots(),
                                                                           robotIndex, left, right, solver.dt());
      const auto & conf = config(robot.name());
      t->resetConfiguration(robot.module().defaultLIPMStabilizerConfiguration());
      t->load(solver, conf);
      t->reset();
      t->setContacts(mc_tasks::lipm_stabilizer::ContactState::DoubleSupport);
      t->staticTarget(robot.com());
      return t;
    });
}
