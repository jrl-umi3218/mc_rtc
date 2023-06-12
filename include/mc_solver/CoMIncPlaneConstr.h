/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_solver/ConstraintSet.h>

#include <mc_rtc/deprecated.h>

#include <mc_rbdyn/polygon_utils.h>

#include <mc_rtc/void_ptr.h>

namespace mc_solver
{

/** \class CoMIncPlaneConstr
 *
 *  For every plane provided to this constraint, this ensures that the robot's
 *  CoM remains on the positive side of this plane.
 *
 */
struct MC_SOLVER_DLLAPI CoMIncPlaneConstr : public ConstraintSet
{
public:
  CoMIncPlaneConstr(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double dt);

  void addToSolverImpl(QPSolver & solver) override;

  void removeFromSolverImpl(QPSolver & solver) override;

  /** Set joints that participate in the constraint */
  void setActiveJoints(const std::vector<std::string> & joints);

  /** Set joints that do not participate in the constraint */
  void setInactiveJoints(const std::vector<std::string> & joints);

  /** Enable all joints for the constraint */
  void resetActiveJoints();

  MC_RTC_DEPRECATED inline void set_planes(QPSolver & solver,
                                           const std::vector<mc_rbdyn::Plane> & planes,
                                           const std::vector<Eigen::Vector3d> & speeds = {},
                                           const std::vector<Eigen::Vector3d> & normalsDots = {},
                                           double iDist = 0.05,
                                           double sDist = 0.01,
                                           double damping = 0.1,
                                           double dampingOff = 0.)
  {
    setPlanes(solver, planes, speeds, normalsDots, iDist, sDist, damping, dampingOff);
  }

  void setPlanes(QPSolver & solver,
                 const std::vector<mc_rbdyn::Plane> & planes,
                 const std::vector<Eigen::Vector3d> & speeds = {},
                 const std::vector<Eigen::Vector3d> & normalsDots = {},
                 double iDist = 0.05,
                 double sDist = 0.01,
                 double damping = 0.1,
                 double dampingOff = 0.);

private:
  /** Holds the constraint implementation
   *
   * In Tasks backend:
   * - tasks::qp::CoMIncPlaneConstr
   *
   * In TVM backend:
   * - details::TVMCoMIncPlaneConstr
   */
  mc_rtc::void_ptr constraint_;
};

} // namespace mc_solver
