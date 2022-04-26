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
    mc_rtc::log::error("Could not add bounded speed constraint for body {} since it does not exist in robot {}",
                       bodyName, solver.robots().robot(robotIndex).name());
  }
}

void BoundedSpeedConstr::addBoundedSpeed(QPSolver & solver,
                                         const mc_rbdyn::RobotFrame & frame,
                                         const Eigen::MatrixXd & dof,
                                         const Eigen::VectorXd & lowerSpeed,
                                         const Eigen::VectorXd & upperSpeed)
{
  if(frame.robot().robotIndex() != robotIndex)
  {
    mc_rtc::log::error(
        "Could not add bounded speed constraint for frame {} since it belongs to {} while this constraint acts on {}",
        frame.name(), frame.robot().name(), solver.robot(robotIndex).name());
    return;
  }
  constr->addBoundedSpeed(solver.robots().mbs(), frame.body(), Eigen::Vector3d::Zero(), dof * frame.X_b_f().matrix(),
                          lowerSpeed, upperSpeed);
  constr->updateBoundedSpeeds();
  solver.updateConstrSize();
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
    mc_rtc::log::error("Could not remove bounded speed constraint for body {} since it does not exist in robot {}",
                       bodyName, solver.robots().robot(robotIndex).name());
  }
  return r;
}

bool BoundedSpeedConstr::removeBoundedSpeed(QPSolver & solver, const mc_rbdyn::RobotFrame & frame)
{
  return removeBoundedSpeed(solver, frame.body());
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

static auto registered = mc_solver::ConstraintSetLoader::register_load_function(
    "boundedSpeed",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      const auto & robot = robotFromConfig(config, solver.robots(), "boundedSpeed");
      auto ret = std::make_shared<mc_solver::BoundedSpeedConstr>(solver.robots(), robot.robotIndex(), solver.dt());
      if(config.has("constraints"))
      {
        for(const auto & c : config("constraints"))
        {
          Eigen::Matrix6d dof = Eigen::Matrix6d::Identity();
          if(c.has("dof"))
          {
            const auto & c_dof = c("dof");
            if(c_dof.size() == 6)
            {
              Eigen::Vector6d v = c_dof;
              dof = v.asDiagonal();
            }
            else
            {
              dof = c_dof;
            }
          }
          Eigen::VectorXd lowerSpeed;
          Eigen::VectorXd upperSpeed = [&]() -> Eigen::VectorXd {
            if(c.has("speed"))
            {
              lowerSpeed = c("speed");
              return lowerSpeed;
            }
            lowerSpeed = c("lowerSpeed");
            return c("upperSpeed");
          }();
          if(lowerSpeed.size() != upperSpeed.size())
          {
            mc_rtc::log::error_and_throw("lowerSpeed size ({}) must match upperSpeed size ({})", lowerSpeed.size(),
                                         upperSpeed.size());
          }
          if(lowerSpeed.size() != dof.rows())
          {
            mc_rtc::log::error_and_throw("dof rows ({}) must match speed size ({})", dof.rows(), lowerSpeed.size());
          }
          if(c.has("body"))
          {
            ret->addBoundedSpeed(solver, c("body"), c("bodyPoint", Eigen::Vector3d::Zero().eval()), dof, lowerSpeed,
                                 upperSpeed);
          }
          else
          {
            ret->addBoundedSpeed(solver, robot.frame(c("frame")), dof, lowerSpeed, upperSpeed);
          }
        }
      }
      return ret;
    });
}
