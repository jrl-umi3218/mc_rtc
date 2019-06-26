/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/polygon_utils.h>
#include <mc_solver/QPSolver.h>
#include <mc_solver/api.h>

#include <Tasks/QPConstr.h>

namespace mc_solver
{

/** \class CoMIncPlaneConstr
 * \brief Wrapper around tasks::qp::CoMIncPlaneConstr
 */
struct MC_SOLVER_DLLAPI CoMIncPlaneConstr : public ConstraintSet
{
public:
  CoMIncPlaneConstr(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double dt);

  virtual void addToSolver(const std::vector<rbd::MultiBody> & mbs, tasks::qp::QPSolver & solver) override;

  virtual void removeFromSolver(tasks::qp::QPSolver & solver) override;

  void set_planes(QPSolver & solver,
                  const std::vector<mc_rbdyn::Plane> & planes,
                  const std::vector<Eigen::Vector3d> & speeds = {},
                  const std::vector<Eigen::Vector3d> & normalsDots = {},
                  double iDist = 0.05,
                  double sDist = 0.01,
                  double damping = 0.1,
                  double dampingOff = 0.);

private:
  std::shared_ptr<tasks::qp::CoMIncPlaneConstr> constr;
};

} // namespace mc_solver
