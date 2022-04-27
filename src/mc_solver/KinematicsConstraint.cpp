/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_solver/KinematicsConstraint.h>

#include <mc_solver/TasksQPSolver.h>

#include <Tasks/Bounds.h>
#include <Tasks/QPConstr.h>

#include <array>

namespace mc_solver
{

static mc_rtc::void_ptr initialize_tasks(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double timeStep)
{
  const mc_rbdyn::Robot & robot = robots.robot(robotIndex);
  tasks::QBound qBound(robot.ql(), robot.qu());
  tasks::AlphaDBound aDBound(robot.al(), robot.au());
  tasks::AlphaDDBound aDDBound(robot.jl(), robot.ju());
  return mc_rtc::make_void_ptr<tasks::qp::JointLimitsConstr>(robots.mbs(), static_cast<int>(robotIndex), qBound,
                                                             aDBound, aDDBound, timeStep);
}

static mc_rtc::void_ptr initialize(QPSolver::Backend backend,
                                   const mc_rbdyn::Robots & robots,
                                   unsigned int robotIndex,
                                   double timeStep)
{
  switch(backend)
  {
    case QPSolver::Backend::Tasks:
      return initialize_tasks(robots, robotIndex, timeStep);
    default:
      mc_rtc::log::error_and_throw("[KinematicsConstraint] Not implemented for solver backend: {}", backend);
  }
}

KinematicsConstraint::KinematicsConstraint(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double timeStep)
: constraint_(initialize(backend_, robots, robotIndex, timeStep))
{
}

static mc_rtc::void_ptr initialize_tasks(const mc_rbdyn::Robots & robots,
                                         unsigned int robotIndex,
                                         double timeStep,
                                         const std::array<double, 3> & damper,
                                         double velocityPercent)
{
  const mc_rbdyn::Robot & robot = robots.robot(robotIndex);
  tasks::QBound qBound(robot.ql(), robot.qu());
  double percentInter = damper[0];
  double percentSecur = damper[1];
  double offset = damper[2];

  std::vector<std::vector<double>> vl = robot.vl();
  std::vector<std::vector<double>> vu = robot.vu();
  std::vector<std::vector<double>> al = robot.al();
  std::vector<std::vector<double>> au = robot.au();
  std::vector<std::vector<double>> jl = robot.jl();
  std::vector<std::vector<double>> ju = robot.ju();
  for(auto & vi : vl)
  {
    for(auto & v : vi)
    {
      v = v * velocityPercent;
    }
  }
  for(auto & vi : vu)
  {
    for(auto & v : vi)
    {
      v = v * velocityPercent;
    }
  }
  tasks::AlphaBound alphaBound(vl, vu);
  tasks::AlphaDBound alphaDBound(al, au);
  tasks::AlphaDDBound alphaDDBound(jl, ju);

  return mc_rtc::make_void_ptr<tasks::qp::DamperJointLimitsConstr>(robots.mbs(), static_cast<int>(robotIndex), qBound,
                                                                   alphaBound, alphaDBound, alphaDDBound, percentInter,
                                                                   percentSecur, offset, timeStep);
}

static mc_rtc::void_ptr initialize(QPSolver::Backend backend,
                                   const mc_rbdyn::Robots & robots,
                                   unsigned int robotIndex,
                                   double timeStep,
                                   const std::array<double, 3> & damper,
                                   double velocityPercent)
{
  switch(backend)
  {
    case QPSolver::Backend::Tasks:
      return initialize_tasks(robots, robotIndex, timeStep, damper, velocityPercent);
    default:
      mc_rtc::log::error_and_throw("[KinematicsConstraint] Not implemented for solver backend: {}", backend);
  }
}

KinematicsConstraint::KinematicsConstraint(const mc_rbdyn::Robots & robots,
                                           unsigned int robotIndex,
                                           double timeStep,
                                           const std::array<double, 3> & damper,
                                           double velocityPercent)
: constraint_(initialize(backend_, robots, robotIndex, timeStep, damper, velocityPercent))
{
}

void KinematicsConstraint::addToSolverImpl(mc_solver::QPSolver & solver)
{
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
      static_cast<tasks::qp::ConstraintFunction<tasks::qp::Bound> *>(constraint_.get())
          ->addToSolver(solver.robots().mbs(), static_cast<mc_solver::TasksQPSolver &>(solver).solver());
      break;
    default:
      break;
  }
}

void KinematicsConstraint::removeFromSolverImpl(mc_solver::QPSolver & solver)
{
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
      static_cast<tasks::qp::ConstraintFunction<tasks::qp::Bound> *>(constraint_.get())
          ->removeFromSolver(static_cast<mc_solver::TasksQPSolver &>(solver).solver());
      break;
    default:
      break;
  }
}

} // namespace mc_solver
