/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_solver/DynamicsConstraint.h>

#include <mc_solver/ConstraintSetLoader.h>
#include <mc_solver/TasksQPSolver.h>

#include <mc_tvm/DynamicFunction.h>

#include <Tasks/Bounds.h>

#include "TVMKinematicsConstraint.h"

namespace mc_solver
{

static mc_rtc::void_ptr initialize_tasks(const mc_rbdyn::Robots & robots,
                                         unsigned int robotIndex,
                                         double timeStep,
                                         bool infTorque)
{
  const auto & robot = robots.robot(robotIndex);
  std::vector<std::vector<double>> tl = robot.tl();
  std::vector<std::vector<double>> tu = robot.tu();
  std::vector<std::vector<double>> tdl = robot.tdl();
  std::vector<std::vector<double>> tdu = robot.tdu();
  if(infTorque)
  {
    for(auto & ti : tl)
    {
      for(auto & t : ti) { t = -INFINITY; }
    }
    for(auto & ti : tu)
    {
      for(auto & t : ti) { t = INFINITY; }
    }
    for(auto & tdi : tdl)
    {
      for(auto & td : tdi) { td = -INFINITY; }
    }
    for(auto & tdi : tdu)
    {
      for(auto & td : tdi) { td = INFINITY; }
    }
  }
  tasks::TorqueBound tBound(tl, tu);
  tasks::TorqueDBound tDBound(tdl, tdu);
  if(robot.flexibility().size() != 0)
  {
    std::vector<tasks::qp::SpringJoint> sjList;
    for(const auto & flex : robot.flexibility())
    {
      sjList.push_back(tasks::qp::SpringJoint(flex.jointName, flex.K, flex.C, flex.O));
    }
    return mc_rtc::make_void_ptr<tasks::qp::MotionSpringConstr>(robots.mbs(), static_cast<int>(robotIndex), tBound,
                                                                tDBound, timeStep, sjList);
  }
  else
  {
    return mc_rtc::make_void_ptr<tasks::qp::MotionConstr>(robots.mbs(), static_cast<int>(robotIndex), tBound, tDBound,
                                                          timeStep);
  }
}

mc_rtc::void_ptr initialize_tvm(const mc_rbdyn::Robot & robot)
{
  return mc_rtc::make_void_ptr<mc_tvm::DynamicFunctionPtr>(std::make_shared<mc_tvm::DynamicFunction>(robot));
}

static mc_rtc::void_ptr initialize(QPSolver::Backend backend,
                                   const mc_rbdyn::Robots & robots,
                                   unsigned int robotIndex,
                                   double timeStep,
                                   bool infTorque)
{
  switch(backend)
  {
    case QPSolver::Backend::Tasks:
      return initialize_tasks(robots, robotIndex, timeStep, infTorque);
    case QPSolver::Backend::TVM:
      return initialize_tvm(robots.robot(robotIndex));
    default:
      mc_rtc::log::error_and_throw("[DynamicsConstraint] Not implemented for solver backend: {}", backend);
  }
}

DynamicsConstraint::DynamicsConstraint(const mc_rbdyn::Robots & robots,
                                       unsigned int robotIndex,
                                       double timeStep,
                                       bool infTorque)
: KinematicsConstraint(robots, robotIndex, timeStep),
  motion_constr_(initialize(backend_, robots, robotIndex, timeStep, infTorque)), robotIndex_(robotIndex)
{
}

DynamicsConstraint::DynamicsConstraint(const mc_rbdyn::Robots & robots,
                                       unsigned int robotIndex,
                                       double timeStep,
                                       const std::array<double, 3> & damper,
                                       double velocityPercent,
                                       bool infTorque)
: KinematicsConstraint(robots, robotIndex, timeStep, damper, velocityPercent),
  motion_constr_(initialize(backend_, robots, robotIndex, timeStep, infTorque)), robotIndex_(robotIndex)
{
}

void DynamicsConstraint::addToSolverImpl(QPSolver & solver)
{
  KinematicsConstraint::addToSolverImpl(solver);
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
      static_cast<tasks::qp::MotionConstr *>(motion_constr_.get())
          ->addToSolver(solver.robots().mbs(), static_cast<TasksQPSolver &>(solver).solver());
      break;
    case QPSolver::Backend::TVM:
    {
      auto & constraints_ = static_cast<TVMKinematicsConstraint *>(constraint_.get())->constraints_;
      auto & problem = tvm_solver(solver).problem();
      auto & tvm_robot = solver.robot(robotIndex_).tvmRobot();
      auto tL = problem.add(tvm_robot.limits().tl <= tvm_robot.tau() <= tvm_robot.limits().tu,
                            tvm::task_dynamics::None(), {tvm::requirements::PriorityLevel(0)});
      constraints_.push_back(tL);
      mc_tvm::DynamicFunctionPtr dyn_fn = *static_cast<mc_tvm::DynamicFunctionPtr *>(motion_constr_.get());
      auto dyn = problem.add(dyn_fn == 0., tvm::task_dynamics::None(), {tvm::requirements::PriorityLevel(0)});
      constraints_.push_back(dyn);
      auto cstr = problem.constraint(*dyn);
      problem.add(tvm::hint::Substitution(cstr, tvm_robot.tau()));
      break;
    }
    default:
      break;
  }
}

void DynamicsConstraint::removeFromSolverImpl(QPSolver & solver)
{
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
    {
      KinematicsConstraint::removeFromSolverImpl(solver);
      static_cast<tasks::qp::MotionConstr *>(motion_constr_.get())
          ->removeFromSolver(static_cast<TasksQPSolver &>(solver).solver());
      break;
    }
    case QPSolver::Backend::TVM:
    {
      auto & constr = *static_cast<TVMKinematicsConstraint *>(constraint_.get());
      auto & problem = tvm_solver(solver).problem();
      problem.removeSubstitutionFor(*problem.constraint(*constr.constraints_.back()));
      KinematicsConstraint::removeFromSolverImpl(solver);
      break;
    }
    default:
      break;
  }
}

} // namespace mc_solver

namespace
{

mc_solver::ConstraintSetPtr load_kin_constr(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  const auto robotIndex = robotIndexFromConfig(config, solver.robots(), "kinematics");
  if(config.has("damper"))
  {
    return std::make_shared<mc_solver::KinematicsConstraint>(solver.robots(), robotIndex, solver.dt(), config("damper"),
                                                             config("velocityPercent", 0.5));
  }
  else
  {
    return std::make_shared<mc_solver::KinematicsConstraint>(solver.robots(), robotIndex, solver.dt());
  }
}

mc_solver::ConstraintSetPtr load_dyn_constr(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  const auto robotIndex = robotIndexFromConfig(config, solver.robots(), "dynamics");
  if(config.has("damper"))
  {
    return std::make_shared<mc_solver::DynamicsConstraint>(solver.robots(), robotIndex, solver.dt(), config("damper"),
                                                           config("velocityPercent", 0.5), config("infTorque", false));
  }
  else
  {
    return std::make_shared<mc_solver::DynamicsConstraint>(solver.robots(), robotIndex, solver.dt(),
                                                           config("infTorque", false));
  }
}

static auto kin_registered = mc_solver::ConstraintSetLoader::register_load_function("kinematics", &load_kin_constr);
static auto dyn_registered = mc_solver::ConstraintSetLoader::register_load_function("dynamics", &load_dyn_constr);

} // namespace
