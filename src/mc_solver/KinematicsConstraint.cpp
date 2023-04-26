/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_solver/KinematicsConstraint.h>

#include <mc_solver/TasksQPSolver.h>

#include <Tasks/Bounds.h>
#include <Tasks/QPConstr.h>

#include <array>

#include "TVMKinematicsConstraint.h"

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

TVMKinematicsConstraint::TVMKinematicsConstraint(const mc_rbdyn::Robot & robot,
                                                 const std::array<double, 3> & damper,
                                                 double vp)
: robot_(robot), damper_(damper), velocityPercent_(vp)
{
}

void TVMKinematicsConstraint::addToSolver(mc_solver::TVMQPSolver & solver)
{
  auto & tvm_robot = robot_.tvmRobot();
  /** Joint limits */
  int startParam = tvm_robot.qFloatingBase()->size();
  auto nParams = tvm_robot.qJoints()->size();
  auto ql = tvm_robot.limits().ql.segment(startParam, nParams);
  auto qu = tvm_robot.limits().qu.segment(startParam, nParams);
  Eigen::VectorXd di = damper_[0] * (qu - ql);
  Eigen::VectorXd ds = damper_[1] * (qu - ql);
  for(int i = 0; i < nParams; ++i)
  {
    if(std::isinf(di(i)))
    {
      di(i) = 0.01;
      ds(i) = 0.005;
    }
  }
  auto jl = solver.problem().add(
      ql <= tvm_robot.qJoints() <= qu,
      tvm::task_dynamics::VelocityDamper(solver.dt(), {di, ds, Eigen::VectorXd::Constant(nParams, 1, 0),
                                                       Eigen::VectorXd::Constant(nParams, 1, damper_[2])}),
      {tvm::requirements::PriorityLevel(0)});
  constraints_.push_back(jl);
  /** Velocity limits */
  int startDof = tvm_robot.qFloatingBase()->space().tSize();
  auto nDof = tvm_robot.qJoints()->space().tSize();
  auto vl = tvm_robot.limits().vl.segment(startDof, nDof) * velocityPercent_;
  auto vu = tvm_robot.limits().vu.segment(startDof, nDof) * velocityPercent_;
  auto vL =
      solver.problem().add(vl <= tvm::dot(tvm_robot.qJoints()) <= vu, tvm::task_dynamics::Proportional(1 / solver.dt()),
                           {tvm::requirements::PriorityLevel(0)});
  constraints_.push_back(vL);
  /** Acceleration limits */
  auto al = tvm_robot.limits().al.segment(startDof, nDof);
  auto au = tvm_robot.limits().au.segment(startDof, nDof);
  auto aL = solver.problem().add(al <= tvm::dot(tvm_robot.qJoints(), 2) <= au, tvm::task_dynamics::None{},
                                 {tvm::requirements::PriorityLevel(0)});
  constraints_.push_back(aL);
  /** Mimic constraints */
  for(const auto & m : tvm_robot.mimics())
  {
    Eigen::DenseIndex startIdx = 0;
    const auto & leader = m.first;
    const auto & followers = m.second.first;
    const auto & mimicMult = m.second.second;
    for(const auto & f : followers)
    {
      Eigen::MatrixXd A = mimicMult.segment(startIdx, f->size());
      auto mimic = solver.problem().add(A * tvm::dot(leader, 2) - tvm::dot(f, 2) == 0., tvm::task_dynamics::None{},
                                        {tvm::requirements::PriorityLevel(0)});
      solver.problem().add(tvm::hint::Substitution(solver.problem().constraint(*mimic), tvm::dot(f, 2),
                                                   tvm::constant::fullRank, tvm::hint::internal::DiagonalCalculator{}));
      mimics_constraints_.push_back(mimic);
      startIdx += f->size();
    }
  }
  /** FIXME Implement torque derivative and jerk bounds */
}

void TVMKinematicsConstraint::removeFromSolver(mc_solver::TVMQPSolver & solver)
{
  for(auto & c : mimics_constraints_)
  {
    solver.problem().removeSubstitutionFor(*solver.problem().constraint(*c));
    solver.problem().remove(*c);
  }
  for(auto & c : constraints_) { solver.problem().remove(*c); }
  constraints_.clear();
  mimics_constraints_.clear();
}

static mc_rtc::void_ptr initialize_tvm(const mc_rbdyn::Robot & robot, const std::array<double, 3> & damper, double vp)
{
  return mc_rtc::make_void_ptr<TVMKinematicsConstraint>(robot, damper, vp);
}

static mc_rtc::void_ptr initialize_tvm(const mc_rbdyn::Robot & robot)
{
  return initialize_tvm(robot, {0.1, 0.01, 0.5}, 0.5);
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
    case QPSolver::Backend::TVM:
      return initialize_tvm(robots.robot(robotIndex));
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
    for(auto & v : vi) { v = v * velocityPercent; }
  }
  for(auto & vi : vu)
  {
    for(auto & v : vi) { v = v * velocityPercent; }
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
    case QPSolver::Backend::TVM:
      return initialize_tvm(robots.robot(robotIndex), damper, velocityPercent);
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
    case QPSolver::Backend::TVM:
      static_cast<TVMKinematicsConstraint *>(constraint_.get())->addToSolver(tvm_solver(solver));
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
    case QPSolver::Backend::TVM:
      static_cast<TVMKinematicsConstraint *>(constraint_.get())->removeFromSolver(tvm_solver(solver));
      break;
    default:
      break;
  }
}

} // namespace mc_solver
