/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/OrientationTask.h>
#include <Eigen/src/Core/Matrix.h>

namespace mc_tasks
{

struct MC_TASKS_DLLAPI CompliantOrientationTask : public OrientationTask
{
public:
  CompliantOrientationTask(const std::string & bodyName_,
                           const mc_rbdyn::Robots & robots,
                           unsigned int robotIndex,
                           double stiffness,
                           double weight);

  /** Change reference acceleration
   *
   * \p refAccel Should be of size nrDof
   */
  void refAccel(const Eigen::Vector3d & refAccel) noexcept;

  // Set task to be compliant or not
  void makeCompliant(bool compliance);
  void setComplianceVector(Eigen::Vector3d gamma);
  // Get compliance state of the task
  bool isCompliant(void);
  Eigen::Vector3d getComplianceVector(void);

  // void load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) override;

protected:
  void addToSolver(mc_solver::QPSolver & solver) override;

  void update(mc_solver::QPSolver & solver) override;

  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;

  Eigen::Matrix3d Gamma_;

  mc_tvm::Robot & tvm_robot_;

  unsigned int rIdx_;

  std::string bodyName_;
  const mc_rbdyn::RobotFrame & frame_;

  rbd::Jacobian * jac_;

  Eigen::Vector3d refAccel_;
};

} // namespace mc_tasks
