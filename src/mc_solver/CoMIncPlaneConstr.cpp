/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_solver/CoMIncPlaneConstr.h>
#include <mc_solver/ConstraintSetLoader.h>

namespace mc_solver
{

CoMIncPlaneConstr::CoMIncPlaneConstr(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double dt)
: constr(new tasks::qp::CoMIncPlaneConstr(robots.mbs(), robotIndex, dt))
{
}

void CoMIncPlaneConstr::addToSolver(const std::vector<rbd::MultiBody> & mbs, tasks::qp::QPSolver & solver)
{
  constr->addToSolver(mbs, solver);
}

void CoMIncPlaneConstr::removeFromSolver(tasks::qp::QPSolver & solver)
{
  constr->removeFromSolver(solver);
}

void CoMIncPlaneConstr::set_planes(QPSolver & solver,
                                   const std::vector<mc_rbdyn::Plane> & planes,
                                   const std::vector<Eigen::Vector3d> & speeds,
                                   const std::vector<Eigen::Vector3d> & normalsDots,
                                   double iDist,
                                   double sDist,
                                   double damping,
                                   double dampingOff)
{
  constr->reset();
  if(speeds.size() != 0 && normalsDots.size() == speeds.size() && planes.size() == speeds.size())
  {
    for(size_t i = 0; i < planes.size(); ++i)
    {
      if(planes[i].normal.norm() > 0.5)
      {
        constr->addPlane(static_cast<int>(i), planes[i].normal, planes[i].offset, iDist, sDist, damping, speeds[i],
                         normalsDots[i], dampingOff);
      }
    }
  }
  else
  {
    if(speeds.size() != 0 && (normalsDots.size() != speeds.size() || planes.size() != speeds.size()))
    {
      // LOG_WARNING("set_planes: speeds size > 0 but different from normalsDots or planes, acting as if speeds were not
      // provided")
    }
    for(size_t i = 0; i < planes.size(); ++i)
    {
      if(planes[i].normal.norm() > 0.5)
      {
        constr->addPlane(static_cast<int>(i), planes[i].normal, planes[i].offset, iDist, sDist, damping, dampingOff);
      }
    }
  }
  constr->updateNrPlanes();
  solver.updateConstrSize();
}

} // namespace mc_solver

namespace
{

static bool registered = mc_solver::ConstraintSetLoader::register_load_function(
    "CoMIncPlane",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      auto ret = std::make_shared<mc_solver::CoMIncPlaneConstr>(solver.robots(), config("robotIndex"), solver.dt());
      return ret;
    });
}
