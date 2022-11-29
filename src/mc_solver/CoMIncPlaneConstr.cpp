/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_solver/CoMIncPlaneConstr.h>

#include <mc_solver/ConstraintSetLoader.h>
#include <mc_solver/TasksQPSolver.h>

#include <Tasks/QPConstr.h>

namespace mc_solver
{

/** Helper to cast the constraint */
static tasks::qp::CoMIncPlaneConstr & tasks_constraint(mc_rtc::void_ptr & ptr)
{
  return *static_cast<tasks::qp::CoMIncPlaneConstr *>(ptr.get());
}

static mc_rtc::void_ptr make_constraint(QPSolver::Backend backend,
                                        const mc_rbdyn::Robots & robots,
                                        unsigned int robotIndex,
                                        double dt)
{
  switch(backend)
  {
    case QPSolver::Backend::Tasks:
      return mc_rtc::make_void_ptr<tasks::qp::CoMIncPlaneConstr>(robots.mbs(), static_cast<int>(robotIndex), dt);
    default:
      mc_rtc::log::error_and_throw("[CoMIncPlaneConstr] Not implemented for solver backend: {}", backend);
  }
}

CoMIncPlaneConstr::CoMIncPlaneConstr(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double dt)
: constraint_(make_constraint(backend_, robots, robotIndex, dt))
{
}

void CoMIncPlaneConstr::addToSolverImpl(QPSolver & solver)
{
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
    {
      auto & qpsolver = tasks_solver(solver).solver();
      tasks_constraint(constraint_).addToSolver(solver.robots().mbs(), qpsolver);
    }
    break;
    default:
      break;
  }
}

void CoMIncPlaneConstr::removeFromSolverImpl(QPSolver & solver)
{
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
      tasks_constraint(constraint_).removeFromSolver(tasks_solver(solver).solver());
      break;
    default:
      break;
  }
}

void CoMIncPlaneConstr::setPlanes(QPSolver & solver,
                                  const std::vector<mc_rbdyn::Plane> & planes,
                                  const std::vector<Eigen::Vector3d> & speeds,
                                  const std::vector<Eigen::Vector3d> & normalsDots,
                                  double iDist,
                                  double sDist,
                                  double damping,
                                  double dampingOff)
{
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
    {
      auto & constr = tasks_constraint(constraint_);
      auto & qpsolver = tasks_solver(solver);
      constr.reset();
      if(speeds.size() != 0 && normalsDots.size() == speeds.size() && planes.size() == speeds.size())
      {
        for(size_t i = 0; i < planes.size(); ++i)
        {
          if(planes[i].normal.norm() > 0.5)
          {
            constr.addPlane(static_cast<int>(i), planes[i].normal, planes[i].offset, iDist, sDist, damping, speeds[i],
                            normalsDots[i], dampingOff);
          }
        }
      }
      else
      {
        if(speeds.size() != 0 && (normalsDots.size() != speeds.size() || planes.size() != speeds.size()))
        {
          mc_rtc::log::warning(
              "set_planes: speeds size > 0 but different from normalsDots or planes, acting as if speeds "
              "were not provided");
        }
        for(size_t i = 0; i < planes.size(); ++i)
        {
          if(planes[i].normal.norm() > 0.5)
          {
            constr.addPlane(static_cast<int>(i), planes[i].normal, planes[i].offset, iDist, sDist, damping, dampingOff);
          }
        }
      }
      constr.updateNrPlanes();
      qpsolver.updateConstrSize();
    }
    break;
    default:
      break;
  }
}

} // namespace mc_solver

namespace
{

static auto registered = mc_solver::ConstraintSetLoader::register_load_function(
    "CoMIncPlane",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      auto ret = std::make_shared<mc_solver::CoMIncPlaneConstr>(
          solver.robots(), robotIndexFromConfig(config, solver.robots(), "CoMIncPlane"), solver.dt());
      return ret;
    });
}
