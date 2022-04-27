/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_solver/BoundedSpeedConstr.h>

#include <mc_solver/ConstraintSetLoader.h>
#include <mc_solver/TasksQPSolver.h>

#include <mc_rtc/logging.h>

#include <Tasks/QPConstr.h>

namespace mc_solver
{

/** Helper to cast the constraint */
static tasks::qp::BoundedSpeedConstr & tasks_constraint(mc_rtc::void_ptr & ptr)
{
  return *static_cast<tasks::qp::BoundedSpeedConstr *>(ptr.get());
}

/** Helper to cast the constraint (const) */
static const tasks::qp::BoundedSpeedConstr & tasks_constraint(const mc_rtc::void_ptr & ptr)
{
  return *static_cast<const tasks::qp::BoundedSpeedConstr *>(ptr.get());
}

static mc_rtc::void_ptr make_constraint(QPSolver::Backend backend,
                                        const mc_rbdyn::Robots & robots,
                                        unsigned int robotIndex,
                                        double dt)
{
  switch(backend)
  {
    case QPSolver::Backend::Tasks:
      return mc_rtc::make_void_ptr<tasks::qp::BoundedSpeedConstr>(robots.mbs(), static_cast<int>(robotIndex), dt);
    default:
      mc_rtc::log::error_and_throw("[BoundedSpeedConstr] Not implemented for solver backend: {}", backend);
  }
}

BoundedSpeedConstr::BoundedSpeedConstr(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double dt)
: constraint_(make_constraint(backend_, robots, robotIndex, dt)), robotIndex(robotIndex)
{
}

void BoundedSpeedConstr::addToSolverImpl(QPSolver & solver)
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

void BoundedSpeedConstr::removeFromSolverImpl(QPSolver & solver)
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

size_t BoundedSpeedConstr::nrBoundedSpeeds() const
{
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
      return tasks_constraint(constraint_).nrBoundedSpeeds();
    default:
      return 0;
  }
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
    switch(backend_)
    {
      case QPSolver::Backend::Tasks:
      {
        auto & qpsolver = tasks_solver(solver);
        auto & constr = tasks_constraint(constraint_);
        constr.addBoundedSpeed(solver.robots().mbs(), bodyName, bodyPoint, dof, lowerSpeed, upperSpeed);
        constr.updateBoundedSpeeds();
        qpsolver.updateConstrSize();
      }
      break;
      default:
        break;
    }
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
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
    {
      auto & constr = tasks_constraint(constraint_);
      auto & qpsolver = tasks_solver(solver);
      constr.addBoundedSpeed(solver.robots().mbs(), frame.body(), Eigen::Vector3d::Zero(), dof * frame.X_b_f().matrix(),
                             lowerSpeed, upperSpeed);
      constr.updateBoundedSpeeds();
      qpsolver.updateConstrSize();
    }
    default:
      break;
  }
}

bool BoundedSpeedConstr::removeBoundedSpeed(QPSolver & solver, const std::string & bodyName)
{
  bool r = false;
  if(solver.robots().robot(robotIndex).hasBody(bodyName))
  {
    switch(backend_)
    {
      case QPSolver::Backend::Tasks:
      {
        auto & constr = tasks_constraint(constraint_);
        auto & qpsolver = tasks_solver(solver);
        r = constr.removeBoundedSpeed(bodyName);
        constr.updateBoundedSpeeds();
        qpsolver.updateConstrSize();
      }
      default:
        break;
    }
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
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
    {
      auto & constr = tasks_constraint(constraint_);
      auto & qpsolver = tasks_solver(solver);
      constr.resetBoundedSpeeds();
      constr.updateBoundedSpeeds();
      qpsolver.updateConstrSize();
    }
    default:
      break;
  }
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
