/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is inspired by Stephane's Caron original implementation as part of
 * lipm_walking_controller <https://github.com/stephane-caron/lipm_walking_controller>
 */

#pragma once

#include <mc_tasks/CoMTask.h>
#include <mc_tasks/CoPTask.h>
#include <mc_tasks/MetaTask.h>

namespace mc_tasks
{

namespace stabilizer
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

/** Walking stabilization based on linear inverted pendulum tracking.
 *
 * Stabilization bridges the gap between the open-loop behavior of the
 * pendulum state reference (feedforward controls) and feedback read from
 * state estimation. In our case, feedback is done on the DCM of the LIPM:
 *
 * \f[
 *   \dot{\xi} = \dot{\xi}^{d} + k_p (\xi^d - \xi) + k_i \int (\xi^d - \xi)
 * \f]
 *
 * Which boils down into corresponding formulas for the CoP and CoM
 * acceleration targets.
 */
struct MC_TASKS_DLLAPI LIPMStabilizerTask : public MetaTask
{

  static constexpr double MAX_AVERAGE_DCM_ERROR = 0.05; /**< Maximum average (integral) DCM error in [m] */
  static constexpr double MAX_COM_ADMITTANCE = 20; /**< Maximum admittance for CoM admittance control */
  static constexpr double MAX_COP_ADMITTANCE = 0.1; /**< Maximum CoP admittance for foot damping control */
  static constexpr double MAX_DCM_D_GAIN = 2.; /**< Maximum DCM derivative gain (no unit) */
  static constexpr double MAX_DCM_I_GAIN = 100.; /**< Maximum DCM average integral gain in [Hz] */
  static constexpr double MAX_DCM_P_GAIN = 20.; /**< Maximum DCM proportional gain in [Hz] */
  static constexpr double MAX_DFZ_ADMITTANCE =
      5e-4; /**< Maximum admittance in [s] / [kg] for foot force difference control */
  static constexpr double MAX_DFZ_DAMPING =
      10.; /**< Maximum normalized damping in [Hz] for foot force difference control */
  static constexpr double MAX_FDC_RX_VEL =
      0.2; /**< Maximum x-axis angular velocity in [rad] / [s] for foot damping control. */
  static constexpr double MAX_FDC_RY_VEL =
      0.2; /**< Maximum y-axis angular velocity in [rad] / [s] for foot damping control. */
  static constexpr double MAX_FDC_RZ_VEL =
      0.2; /**< Maximum z-axis angular velocity in [rad] / [s] for foot damping control. */
  static constexpr double MAX_ZMPCC_COM_OFFSET = 0.05; /**< Maximum CoM offset due to admittance control in [m] */
  static constexpr double MIN_DS_PRESSURE = 15.; /**< Minimum normal contact force in DSP, used to avoid low-pressure
                                                    targets when close to contact switches. */

public:
  LIPMStabilizerTask(const mc_rbdyn::Robots & robots,
                     unsigned int robotIndex,
                     const std::string & leftFootSurface,
                     const std::string & rightFootSurface);
  ~LIPMStabilizerTask() override;

  void reset() override;

  void dimWeight(const Eigen::VectorXd & dimW) override;
  Eigen::VectorXd dimWeight() const override;

  void selectActiveJoints(mc_solver::QPSolver & solver,
                          const std::vector<std::string> & activeJointsName,
                          const std::map<std::string, std::vector<std::array<int, 2>>> & activeDofs = {}) override;

  virtual void selectUnactiveJoints(
      mc_solver::QPSolver & solver,
      const std::vector<std::string> & unactiveJointsName,
      const std::map<std::string, std::vector<std::array<int, 2>>> & unactiveDofs = {}) override;

  void resetJointsSelector(mc_solver::QPSolver & solver) override;

  /*! \brief Returns the task error
   *
   * The vector's dimensions depend on the underlying task
   *
   */
  Eigen::VectorXd eval() const override;

  /*! \brief Returns the task velocity
   *
   * The vector's dimensions depend on the underlying task
   *
   */
  Eigen::VectorXd speed() const override;

  /*! \brief Load parameters from a Configuration object */
  void load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config);

protected:
  void addToSolver(mc_solver::QPSolver & solver) override;
  void removeFromSolver(mc_solver::QPSolver & solver) override;
  void update() override;
  void addToLogger(mc_rtc::Logger &);
  void removeFromLogger(mc_rtc::Logger &);
  void addToGUI(mc_rtc::gui::StateBuilder &);
  void removeFromGUI(mc_rtc::gui::StateBuilder &);

protected:
  std::shared_ptr<mc_tasks::CoMTask> comTask;
  std::shared_ptr<mc_tasks::force::CoPTask> leftFootTask;
  std::shared_ptr<mc_tasks::force::CoPTask> rightFootTask;
  const mc_rbdyn::Robots & robots_;
  unsigned int robotIndex_;

protected:
  Eigen::Vector3d gravity_; /**< Gravity vector from mbc().gravity */
  Eigen::Vector3d vertical_; /**< Vertical vector (normalized gravity) */
};

using LIPMStabilizerTaskPtr = std::shared_ptr<LIPMStabilizerTask>;

} // namespace stabilizer
} // namespace mc_tasks

namespace mc_rtc
{
using Sole = mc_tasks::stabilizer::Sole;

template<>
struct ConfigurationLoader<mc_tasks::stabilizer::Sole>
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
} // namespace mc_rtc
