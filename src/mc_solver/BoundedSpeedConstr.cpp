/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/logging.h>
#include <mc_solver/BoundedSpeedConstr.h>
#include <mc_solver/ConstraintSetLoader.h>

namespace mc_solver
{

BoundedSpeedConstr::BoundedSpeedConstr(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double dt)
: constr(new tasks::qp::BoundedSpeedConstr(robots.mbs(), static_cast<int>(robotIndex), dt)), robotIndex(robotIndex)
{
}

void BoundedSpeedConstr::addToSolver(const std::vector<rbd::MultiBody> & mbs, tasks::qp::QPSolver & solver)
{
  constr->addToSolver(mbs, solver);
}

void BoundedSpeedConstr::removeFromSolver(tasks::qp::QPSolver & solver)
{
  constr->removeFromSolver(solver);
}

size_t BoundedSpeedConstr::nrBoundedSpeeds() const
{
  return constr->nrBoundedSpeeds();
}

void BoundedSpeedConstr::addBoundedSpeed(QPSolver & solver,
                                         const std::string & bodyName,
                                         const Eigen::Vector3d & bodyPoint,
                                         const Eigen::MatrixXd & dof,
                                         const Eigen::VectorXd & speed)
{
  addBoundedSpeed(solver, bodyName, bodyPoint, dof, speed, speed);
}

void BoundedSpeedConstr::addBoundedSpeed(QPSolver & solver,
                                         const std::string & bodyName,
                                         const Eigen::Vector3d & bodyPoint,
                                         const Eigen::MatrixXd & dof,
                                         const Eigen::VectorXd & lowerSpeed,
                                         const Eigen::VectorXd & upperSpeed)
{
  if(solver.robots().robot(robotIndex).hasBody(bodyName))
  {
    constr->addBoundedSpeed(solver.robots().mbs(), bodyName, bodyPoint, dof, lowerSpeed, upperSpeed);
    constr->updateBoundedSpeeds();
    solver.updateConstrSize();
  }
  else
  {
    LOG_ERROR("Could not add bounded speed constraint for body " << bodyName << " since it does not exist in robot "
                                                                 << solver.robots().robot(robotIndex).name())
  }
}

bool BoundedSpeedConstr::removeBoundedSpeed(QPSolver & solver, const std::string & bodyName)
{
  bool r = false;
  if(solver.robots().robot(robotIndex).hasBody(bodyName))
  {
    r = constr->removeBoundedSpeed(bodyName);
    constr->updateBoundedSpeeds();
    solver.updateConstrSize();
  }
  else
  {
    LOG_ERROR("Could not remove bounded speed constraint for body " << bodyName << " since it does not exist in robot "
                                                                    << solver.robots().robot(robotIndex).name())
  }
  return r;
}

void BoundedSpeedConstr::reset(QPSolver & solver)
{
  constr->resetBoundedSpeeds();
  constr->updateBoundedSpeeds();
  solver.updateConstrSize();
}

} // namespace mc_solver

namespace
{

static bool registered = mc_solver::ConstraintSetLoader::register_load_function(
    "boundedSpeed",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      auto ret = std::make_shared<mc_solver::BoundedSpeedConstr>(solver.robots(), config("robotIndex"), solver.dt());
      if(config.has("constraints"))
      {
        for(const auto & c : config("constraints"))
        {
          std::string bName = c("body");
          Eigen::Vector3d bPoint = c("bodyPoint", Eigen::Vector3d::Zero().eval());
          Eigen::Matrix6d dof = c("dof", Eigen::Matrix6d::Identity().eval());
          if(c.has("speed"))
          {
            ret->addBoundedSpeed(solver, bName, bPoint, dof, c("speed"));
          }
          else if(c.has("lowerSpeed"))
          {
            assert(c.has("upperSpeed"));
            ret->addBoundedSpeed(solver, bName, bPoint, dof, c("lowerSpeed"), c("upperSpeed"));
          }
          else
          {
            LOG_ERROR("No speed or lowerSpeed/upperSpeed entry for bounded speed constraint on " << bName)
          }
        }
      }
      return ret;
    });
}
