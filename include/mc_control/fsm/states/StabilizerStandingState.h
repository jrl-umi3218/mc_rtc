/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/fsm/State.h>
#include <mc_planning/Pendulum.h>
#include <mc_tasks/lipm_stabilizer/Contact.h>

namespace mc_tasks
{
namespace lipm_stabilizer
{
struct StabilizerTask;
}
} // namespace mc_tasks

namespace mc_control
{
namespace fsm
{

/**
 * @brief Simple state to control and stabilize the CoM of a biped-like robot
 * using the LIPMStabilizer
 *
 * This state allows to:
 * - Move the CoM to a desired position using the LIPMStabilizer
 *   The reference trajectory is computed as a simple spring-damper system.
 *
 * Typical uses:
 * - Simple stabilized CoM motion
 * - Stabilized manipulation can be easily achieved by putting this state in
 *   parallel to manipulation states
 *
 *  \see mc_tasks::lipm_stabilizer::StabilizerTask
 */
struct StabilizerStandingState : State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(Controller &) override;

  bool run(Controller &) override;

  void teardown(Controller &) override;

protected:
  /**
   * @brief Targets a given Cop.
   * The CoM height will be automatically set at comHeight_ above this CoP.
   * The default CoM height is that of the stabilizer's configuration, and can be
   * overriden from configuration through "comHeight: 0.8" entry.
   *
   * @param cop Desired CoP
   */
  void targetCoP(const Eigen::Vector3d & cop);

  /**
   * @brief Freely target a CoM position.
   * The corresponding CoP position will be computed as the projection of the
   * CoM at the average height of both feet contacts.
   *
   * \note Caution, this function does not check the feasibility of the provided
   * CoM
   *
   * @param com Desired CoM position
   */
  void targetCoM(const Eigen::Vector3d & com);

  /**
   * @brief CoP height from contacts
   * - double support: average of both contact heights
   * - single support: contact height
   */
  double copHeight() const;

protected:
  std::shared_ptr<mc_tasks::lipm_stabilizer::StabilizerTask> stabilizerTask_ = nullptr;
  mc_rtc::Configuration config_; /**< Full state configuration */
  double hasCompletion_ = false; /**< If the latest definition of the state configuration has an empty "completion"
                                   element, no completion rule will be used */

  Eigen::Vector3d dcmThreshold_ = Eigen::Vector3d{0.01, 0.01, 0.01}; /**< Completion criteria threshold */

  mc_planning::Pendulum pendulum_; /** LIPM Pendulum model */

  double K_ = 5; /**< CoM tracking stiffness (set-point) */
  double D_ = 0.; /**< CoM tracking damping */

  double comHeight_ = 0; /**< Desired height of the CoM above the CoP target */
  Eigen::Vector3d comTarget_ = Eigen::Vector3d::Zero(); /**< World target for the CoM */
  Eigen::Vector3d copTarget_ = Eigen::Vector3d::Zero(); /**< World target for the CoP */

  bool optionalGUI_ = true; /**< Controls whether optional GUI elements are displayed */
};

} // namespace fsm
} // namespace mc_control
