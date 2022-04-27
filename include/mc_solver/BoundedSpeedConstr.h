/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_solver/ConstraintSet.h>

#include <mc_rtc/void_ptr.h>

namespace mc_solver
{

/** \class BoundedSpeedConstr
 *
 * Manage constraints on the speed of a given frame
 */
struct MC_SOLVER_DLLAPI BoundedSpeedConstr : public ConstraintSet
{
public:
  BoundedSpeedConstr(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double dt);

  void addToSolverImpl(QPSolver & solver) override;

  void removeFromSolverImpl(QPSolver & solver) override;

  /** Add a fixed speed constraint on the given body
   *
   * \param solver Solver where this constraint exists
   *
   * \param bodyName Body affected by this constraint
   *
   * \param bodyPoint Point on the given body to affect
   *
   * \param dof Can select degrees of freedom, must be of size (6, n), 1 <= n <= 6
   *
   * \param speed Target speed on the selected dof, must of be of size n
   */
  inline void addBoundedSpeed(QPSolver & solver,
                              const std::string & bodyName,
                              const Eigen::Vector3d & bodyPoint,
                              const Eigen::MatrixXd & dof,
                              const Eigen::VectorXd & speed)
  {
    addBoundedSpeed(solver, bodyName, bodyPoint, dof, speed, speed);
  }

  /** Add a bounded speed constraint on the given body
   *
   * \param solver Solver where this constraint exists
   *
   * \param bodyName Body affected by this constraint
   *
   * \param bodyPoint Point on the given body to affect
   *
   * \param dof Can select degrees of freedom, must be of size (6, n), 1 <= n <= 6
   *
   * \param lowerSpeed Lower bound speed on the selected dof, must of be of size n
   *
   * \param upperSpeed Upper bound speed on the selected dof, must of be of size n
   */
  void addBoundedSpeed(QPSolver & solver,
                       const std::string & bodyName,
                       const Eigen::Vector3d & bodyPoint,
                       const Eigen::MatrixXd & dof,
                       const Eigen::VectorXd & lowerSpeed,
                       const Eigen::VectorXd & upperSpeed);

  /** Add a fixed speed constraint on the given frame
   *
   * \param solver Solver where this constraint exists
   *
   * \param frame Frame affected by this constraint
   *
   * \param dof Can select degrees of freedom, must be of size (6, n), 1 <= n <= 6
   *
   * \param speed Target speed on the selected dof, must of be of size n
   */
  inline void addBoundedSpeed(QPSolver & solver,
                              const mc_rbdyn::RobotFrame & frame,
                              const Eigen::MatrixXd & dof,
                              const Eigen::VectorXd & speed)
  {
    addBoundedSpeed(solver, frame, dof, speed, speed);
  }

  /** Add a fixed speed constraint on the given frame
   *
   * \param solver Solver where this constraint exists
   *
   * \param frame Frame affected by this constraint
   *
   * \param dof Can select degrees of freedom, must be of size (6, n), 1 <= n <= 6
   *
   * \param lowerSpeed Lower bound speed on the selected dof, must of be of size n
   *
   * \param upperSpeed Upper bound speed on the selected dof, must of be of size n
   */
  void addBoundedSpeed(QPSolver & solver,
                       const mc_rbdyn::RobotFrame & frame,
                       const Eigen::MatrixXd & dof,
                       const Eigen::VectorXd & lowerSpeed,
                       const Eigen::VectorXd & upperSpeed);

  /** Remove all constraints associated to a body */
  bool removeBoundedSpeed(QPSolver & solver, const std::string & bodyName);

  /** Remove all constraints associated to a frame */
  bool removeBoundedSpeed(QPSolver & solver, const mc_rbdyn::RobotFrame & frame);

  size_t nrBoundedSpeeds() const;

  void reset(QPSolver & solver);

private:
  /** Holds the constraint implementation
   *
   * In Tasks backend:
   * - tasks::qp::BoundedSpeedConstr
   */
  mc_rtc::void_ptr constraint_;
  unsigned int robotIndex;
};

} // namespace mc_solver
