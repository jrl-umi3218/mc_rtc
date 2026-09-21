/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_solver/DistanceConstraint.h>

namespace mc_solver
{

/**
 * \class CollisionsConstraint
 *
 * \brief Deprecated compatibility wrapper around DistanceConstraint.
 *
 * \deprecated Use DistanceConstraint instead.
 */
struct MC_SOLVER_DLLAPI [[deprecated("Use DistanceConstraint instead.")]] CollisionsConstraint
: public DistanceConstraint
{
public:
  using DistanceConstraint::DistanceConstraint;

  bool removeCollision(QPSolver & solver, const std::string & b1Name, const std::string & b2Name);

  void removeCollisions(QPSolver & solver, const std::vector<mc_rbdyn::DistanceLimit> & cols);

  bool removeCollisionByBody(QPSolver & solver, const std::string & b1Name, const std::string & b2Name);

  void addCollision(QPSolver & solver, const mc_rbdyn::DistanceLimit & col);

  void addCollisions(QPSolver & solver, const std::vector<mc_rbdyn::DistanceLimit> & cols);

  bool hasCollision(const std::string & c1, const std::string & c2) const noexcept;
};

} // namespace mc_solver
