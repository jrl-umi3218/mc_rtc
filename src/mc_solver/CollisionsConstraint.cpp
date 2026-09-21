/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_solver/CollisionsConstraint.h>

namespace mc_solver
{

bool CollisionsConstraint::removeCollision(QPSolver & solver, const std::string & b1Name, const std::string & b2Name)
{
  return removeDistanceLimitByBody(solver, b1Name, b2Name);
}

void CollisionsConstraint::removeCollisions(QPSolver & solver, const std::vector<mc_rbdyn::DistanceLimit> & cols)
{
  removeDistanceLimits(solver, cols);
}

bool CollisionsConstraint::removeCollisionByBody(QPSolver & solver,
                                                 const std::string & b1Name,
                                                 const std::string & b2Name)
{
  return removeDistanceLimitByBody(solver, b1Name, b2Name);
}

void CollisionsConstraint::addCollision(QPSolver & solver, const mc_rbdyn::DistanceLimit & col)
{
  addDistanceLimit(solver, col);
}

void CollisionsConstraint::addCollisions(QPSolver & solver, const std::vector<mc_rbdyn::DistanceLimit> & cols)
{
  addDistanceLimits(solver, cols);
}

bool CollisionsConstraint::hasCollision(const std::string & c1, const std::string & c2) const noexcept
{
  return hasDistanceLimit(c1, c2);
}

} // namespace mc_solver
