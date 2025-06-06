/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/PostureTask.h>

namespace mc_tasks
{

struct MC_TASKS_DLLAPI CompliantPostureTask : public PostureTask
{
public:
  CompliantPostureTask(const mc_solver::QPSolver & solver, unsigned int rIndex, double stiffness, double weight);

  /** Change reference acceleration
   *
   * \p refAccel Should be of size nrDof
   */
  void refAccel(const Eigen::VectorXd & refAccel) noexcept;

  // Set task to be compliant or not
  void makeCompliant(bool compliance);
  void makeCompliant(Eigen::VectorXd gamma);
  // Get compliance state of the task
  bool isCompliant(void);

protected:
  void update(mc_solver::QPSolver & solver);

  void addToGUI(mc_rtc::gui::StateBuilder & gui);

  Eigen::VectorXd gamma_;

  mc_tvm::Robot & tvm_robot_;

  Eigen::VectorXd refAccel_;
};

} // namespace mc_tasks
