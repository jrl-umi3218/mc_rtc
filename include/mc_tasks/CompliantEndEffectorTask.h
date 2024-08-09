/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/EndEffectorTask.h>
#include <RBDyn/MultiBody.h>
#include <Eigen/src/Core/Matrix.h>

namespace mc_tasks
{

/*! \brief Controls an end-effector
 *
 * This task is a thin wrapper around the appropriate tasks in Tasks.
 * The task objective is given in the world frame. For relative control
 * see mc_tasks::RelativeCompliantEndEffectorTask
 */
struct MC_TASKS_DLLAPI CompliantEndEffectorTask : public EndEffectorTask
{
public:
  /*! \brief Constructor
   *
   * \param bodyName Name of the body to control
   *
   * \param robots Robots controlled by this task
   *
   * \param robotIndex Index of the robot controlled by this task
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   *
   */
  CompliantEndEffectorTask(const std::string & bodyName,
                           const mc_rbdyn::Robots & robots,
                           unsigned int robotIndex,
                           double stiffness,
                           double weight);

  /** Change acceleration
   *
   * \p refAccel Should be of size 6
   */
  void refAccel(const Eigen::Vector6d & refAccel) noexcept;

  // Set the compliant behavior of the task
  void makeCompliant(bool compliance);
  void setComplianceVector(Eigen::Vector6d gamma);

  // Get compliance state of the task
  bool isCompliant(void);
  Eigen::Vector6d getComplianceVector(void);

protected:
  void addToSolver(mc_solver::QPSolver & solver);

  void update(mc_solver::QPSolver & solver);

  void addToGUI(mc_rtc::gui::StateBuilder & gui);

  Eigen::MatrixXd compliant_matrix_;

  mc_tvm::Robot * tvm_robot_;

  unsigned int rIdx_;

  std::string bodyName_;

  rbd::Jacobian * jac_;

  Eigen::Vector6d refAccel_;
};

} // namespace mc_tasks
