/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/configuration_io.h>
#include <mc_solver/CollisionsConstraint.h>
#include <mc_solver/ConstraintSetLoader.h>

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

void CollisionsConstraint::removeCollisions(QPSolver & solver, const std::vector<mc_rbdyn::Collision> & cols)
{
  for(const auto & col : cols) { removeDistanceLimit(solver, col); }
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

void CollisionsConstraint::addCollisions(QPSolver & solver, const std::vector<mc_rbdyn::Collision> & cols)
{
  for(const auto & col : cols) { addDistanceLimit(solver, col); }
}

bool CollisionsConstraint::hasCollision(const std::string & c1, const std::string & c2) const noexcept
{
  return hasDistanceLimit(c1, c2);
}

} // namespace mc_solver

namespace
{
static auto registered_collision = mc_solver::ConstraintSetLoader::register_load_function(
    "collision",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
    {
      auto ret = std::make_shared<mc_solver::CollisionsConstraint>(
          solver.robots(), robotIndexFromConfig(config, solver.robots(), "collision", false, "r1Index", "r1", ""),
          robotIndexFromConfig(config, solver.robots(), "distance", false, "r2Index", "r2", ""), solver.dt());
      ret->automaticMonitor(config("automaticMonitor", true));
      if(ret->r1Index == ret->r2Index)
      {
        if(config("useCommon", false))
        {
          ret->addCollisions(solver, solver.robots().robotModule(ret->r1Index).commonDistanceLimits());
          ret->addCollisions(solver, solver.robots().robotModule(ret->r1Index).commonSelfCollisions());
        }
        else if(config("useMinimal", false))
        {
          ret->addCollisions(solver, solver.robots().robotModule(ret->r1Index).minimalDistanceLimits());
          ret->addCollisions(solver, solver.robots().robotModule(ret->r1Index).minimalSelfCollisions());
        }
      }
      std::vector<mc_rbdyn::Collision> distLims = config("collisions", std::vector<mc_rbdyn::Collision>{});
      ret->addCollisions(solver, distLims);
      return ret;
    });
} // namespace
