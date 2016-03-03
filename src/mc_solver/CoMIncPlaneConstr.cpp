#include <mc_solver/CoMIncPlaneConstr.h>

namespace mc_solver
{

CoMIncPlaneConstr::CoMIncPlaneConstr(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double dt)
: constr(new tasks::qp::CoMIncPlaneConstr(robots.mbs(), robotIndex, dt))
{
}

void CoMIncPlaneConstr::addToSolver(const std::vector<rbd::MultiBody> & mbs, tasks::qp::QPSolver & solver) const
{
  constr->addToSolver(mbs, solver);
}

void CoMIncPlaneConstr::removeFromSolver(tasks::qp::QPSolver & solver) const
{
  constr->removeFromSolver(solver);
}

void CoMIncPlaneConstr::set_planes(QPSolver & solver, const std::vector<mc_rbdyn::Plane> & planes, const std::vector<Eigen::Vector3d> & speeds, const std::vector<Eigen::Vector3d> & normalsDots)
{
  constr->reset();
  if(speeds.size() != 0 && normalsDots.size() == speeds.size() && planes.size() == speeds.size())
  {
    for(size_t i = 0; i < planes.size(); ++i)
    {
      if(planes[i].normal.norm() > 0.5)
      {
        constr->addPlane(static_cast<int>(i), planes[i].normal, planes[i].offset, 0.05, 0.01, 0.1, speeds[i], normalsDots[i], 0.);
      }
    }
  }
  else
  {
    if(speeds.size() != 0 && (normalsDots.size() != speeds.size()
                               || planes.size() != speeds.size()))
    {
      //LOG_WARNING("set_planes: speeds size > 0 but different from normalsDots or planes, acting as if speeds were not provided")
    }
    for(size_t i = 0; i < planes.size(); ++i)
    {
      if(planes[i].normal.norm() > 0.5)
      {
        constr->addPlane(static_cast<int>(i), planes[i].normal, planes[i].offset, 0.04, 0.01, 0.01, 0.);
      }
    }
  }
  constr->updateNrPlanes();
  solver.updateConstrSize();
}

} // namespace mc_solver
